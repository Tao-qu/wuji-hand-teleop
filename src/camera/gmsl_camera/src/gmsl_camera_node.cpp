#include "gmsl_camera/gmsl_camera_node.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>

namespace gmsl_camera {
namespace {
sensor_msgs::msg::CameraInfo load_info(const std::string& path, int width, int height) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  if (path.empty()) return info;
  const auto data = YAML::LoadFile(path);
  if (data["image_width"].as<int>() != width || data["image_height"].as<int>() != height) {
    throw std::runtime_error("Calibration dimensions do not match capture: " + path);
  }
  info.distortion_model = data["distortion_model"].as<std::string>();
  info.d = data["distortion_coefficients"]["data"].as<std::vector<double>>();
  const auto assign = [&](const char* key, auto& target) {
    const auto values = data[key]["data"].as<std::vector<double>>();
    if (values.size() != target.size()) throw std::runtime_error("Invalid matrix in " + path);
    std::copy(values.begin(), values.end(), target.begin());
  };
  assign("camera_matrix", info.k);
  assign("rectification_matrix", info.r);
  assign("projection_matrix", info.p);
  return info;
}
void field(diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key,
           const std::string& value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}
}  // namespace

GmslCameraNode::GmslCameraNode() : Node("gmsl", "/cameras") {
  const auto names = declare_parameter<std::vector<std::string>>(
      "cameras", {"head_left", "head_right", "left_wrist", "right_wrist"});
  const int width = declare_parameter("width", 2560);
  const int height = declare_parameter("height", 1984);
  const int fps = declare_parameter("fps", 30);
  preview_width_ = declare_parameter("preview.width", 640);
  preview_fps_ = declare_parameter("preview.fps", 10.0);
  preview_quality_ = declare_parameter("preview.jpeg_quality", 80);
  if (width <= 0 || height <= 0 || fps <= 0 || preview_width_ <= 0 || preview_fps_ <= 0) {
    throw std::runtime_error("Image dimensions and frame rates must be positive");
  }
  const auto qos = rclcpp::SensorDataQoS().keep_last(2);
  for (const auto& name : names) {
    auto channel = std::make_unique<Channel>();
    channel->name = name;
    channel->sensor_id = declare_parameter(name + ".sensor_id", 0);
    channel->frame_id = declare_parameter(name + ".frame_id", name + "_optical_frame");
    const auto calibration = declare_parameter(name + ".calibration_file", std::string{});
    channel->info = load_info(calibration, width, height);
    channel->image_pub = create_publisher<sensor_msgs::msg::Image>(name + "/image_raw", qos);
    channel->info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(name + "/camera_info", qos);
    channel->preview_pub = create_publisher<sensor_msgs::msg::CompressedImage>(
        name + "/preview/compressed", qos);
    channel->capture = std::make_unique<CaptureBackend>(channel->sensor_id, width, height, fps);
    RCLCPP_INFO(get_logger(), "%s: CSI %d, %dx%d @ %d fps; calibration %s", name.c_str(),
                channel->sensor_id, width, height, fps, channel->info.k[0] == 0.0 ? "unset" : "loaded");
    channels_.push_back(std::move(channel));
  }
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  last_report_ = std::chrono::steady_clock::now();
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_diagnostics(); });
  // Start workers only after construction of all channels succeeds.
  try {
    for (auto& channel : channels_) {
      channel->worker = std::thread([this, ptr = channel.get()] { run(*ptr); });
    }
  } catch (...) {
    running_ = false;
    for (auto& channel : channels_) if (channel->worker.joinable()) channel->worker.join();
    throw;
  }
}

GmslCameraNode::~GmslCameraNode() {
  running_ = false;
  for (auto& channel : channels_) if (channel->worker.joinable()) channel->worker.join();
}

void GmslCameraNode::run(Channel& channel) {
  try {
    channel.capture->start();
    auto last_preview = std::chrono::steady_clock::time_point{};
    while (running_ && rclcpp::ok()) {
      Frame frame;
      if (!channel.capture->capture(frame)) continue;
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      message->header.stamp = rclcpp::Time(frame.stamp_ns);
      message->header.frame_id = channel.frame_id;
      message->width = frame.width;
      message->height = frame.height;
      message->encoding = "rgb8";
      message->step = frame.width * 3;
      message->data = std::move(frame.rgb);
      auto info = channel.info;
      info.header = message->header;
      channel.info_pub->publish(info);
      const auto current = std::chrono::steady_clock::now();
      if (channel.preview_pub->get_subscription_count() > 0 &&
          std::chrono::duration<double>(current - last_preview).count() >= 1.0 / preview_fps_) {
        cv::Mat full(message->height, message->width, CV_8UC3, message->data.data());
        cv::Mat reduced, bgr;
        const int output_width = std::min(preview_width_, static_cast<int>(message->width));
        const int output_height = std::max(1, static_cast<int>(
            static_cast<double>(message->height) * output_width / message->width));
        cv::resize(full, reduced, cv::Size(output_width, output_height), 0, 0, cv::INTER_AREA);
        cv::cvtColor(reduced, bgr, cv::COLOR_RGB2BGR);
        sensor_msgs::msg::CompressedImage preview;
        preview.header = message->header;
        preview.format = "rgb8; jpeg compressed bgr8";
        cv::imencode(".jpg", bgr, preview.data, {cv::IMWRITE_JPEG_QUALITY, preview_quality_});
        channel.preview_pub->publish(preview);
        last_preview = current;
      }
      channel.image_pub->publish(std::move(message));
      std::lock_guard<std::mutex> lock(channel.mutex);
      ++channel.frames;
      channel.last_stamp_ns = frame.stamp_ns;
      channel.source_pts = frame.source_pts;
    }
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(channel.mutex);
    channel.error = error.what();
    RCLCPP_ERROR(get_logger(), "%s: %s", channel.name.c_str(), error.what());
  }
  // Drain each Argus pipeline on its worker so all four close in parallel.
  channel.capture.reset();
}

void GmslCameraNode::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = now();
  const auto current = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(current - last_report_).count();
  for (auto& channel : channels_) {
    std::lock_guard<std::mutex> lock(channel->mutex);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "gmsl/" + channel->name;
    status.hardware_id = "csi://" + std::to_string(channel->sensor_id);
    const auto received = channel->frames - channel->last_report_frames;
    status.level = !channel->error.empty() ? status.ERROR : (received ? status.OK : status.WARN);
    status.message = !channel->error.empty() ? channel->error : (received ? "Streaming" : "Waiting for frames");
    field(status, "fps", std::to_string(received / elapsed));
    field(status, "frames", std::to_string(channel->frames));
    field(status, "stamp_source", channel->source_pts ? "gstreamer_pts_to_system_time" : "system_receive_time");
    field(status, "last_frame_stamp_ns", std::to_string(channel->last_stamp_ns));
    field(status, "calibrated", channel->info.k[0] == 0.0 ? "false" : "true");
    channel->last_report_frames = channel->frames;
    message.status.push_back(std::move(status));
  }
  last_report_ = current;
  diagnostics_pub_->publish(message);
}
}  // namespace gmsl_camera

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gmsl_camera::GmslCameraNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("gmsl"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
