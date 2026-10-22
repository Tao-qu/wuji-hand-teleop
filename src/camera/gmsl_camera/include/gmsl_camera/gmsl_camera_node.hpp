#pragma once

#include "gmsl_camera/capture_backend.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace gmsl_camera {
class GmslCameraNode : public rclcpp::Node {
 public:
  GmslCameraNode();
  ~GmslCameraNode() override;

 private:
  struct Channel {
    std::string name;
    std::string frame_id;
    int sensor_id = 0;
    sensor_msgs::msg::CameraInfo info;
    std::unique_ptr<CaptureBackend> capture;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub;
    std::thread worker;
    std::mutex mutex;
    std::string error;
    uint64_t frames = 0;
    uint64_t last_report_frames = 0;
    int64_t last_stamp_ns = 0;
    bool source_pts = false;
  };
  void run(Channel& channel);
  void publish_diagnostics();
  std::atomic<bool> running_{true};
  std::vector<std::unique_ptr<Channel>> channels_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  int preview_width_ = 640;
  int preview_quality_ = 80;
  double preview_fps_ = 10.0;
  std::chrono::steady_clock::time_point last_report_;
};
}  // namespace gmsl_camera
