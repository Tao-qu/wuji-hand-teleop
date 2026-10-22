#include "gmsl_camera/capture_backend.hpp"

#include <gst/video/video.h>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>

namespace gmsl_camera {
namespace {
int64_t realtime_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

CaptureBackend::CaptureBackend(int sensor_id, int width, int height, int fps) {
  gst_init(nullptr, nullptr);
  const std::string description =
      "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) +
      " sensor-mode=0 ! video/x-raw(memory:NVMM),format=NV12,width=" + std::to_string(width) +
      ",height=" + std::to_string(height) + ",framerate=" + std::to_string(fps) +
      "/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
      "video/x-raw,format=RGB ! appsink name=frames max-buffers=2 drop=true sync=false";
  GError* error = nullptr;
  pipeline_ = gst_parse_launch(description.c_str(), &error);
  if (error) {
    const std::string message = error->message;
    g_error_free(error);
    if (pipeline_) gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    throw std::runtime_error(message);
  }
  if (!pipeline_) throw std::runtime_error("Cannot create Argus pipeline");
  sink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "frames"));
}

CaptureBackend::~CaptureBackend() {
  if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
  if (sink_) gst_object_unref(sink_);
  if (pipeline_) gst_object_unref(pipeline_);
}

void CaptureBackend::start() {
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    check_bus();
    throw std::runtime_error("Cannot start Argus pipeline");
  }
}

void CaptureBackend::check_bus() {
  GstBus* bus = gst_element_get_bus(pipeline_);
  GstMessage* message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  gst_object_unref(bus);
  if (!message) return;
  std::string detail = "Camera stream ended";
  if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
    GError* error = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    detail = error ? error->message : "Camera pipeline error";
    if (error) g_error_free(error);
    g_free(debug);
  }
  gst_message_unref(message);
  throw std::runtime_error(detail);
}

bool CaptureBackend::capture(Frame& frame) {
  std::unique_ptr<GstSample, decltype(&gst_sample_unref)> sample(
      gst_app_sink_try_pull_sample(sink_, 200 * GST_MSECOND), gst_sample_unref);
  if (!sample) {
    check_bus();
    return false;
  }
  GstBuffer* buffer = gst_sample_get_buffer(sample.get());
  frame.stamp_ns = realtime_ns();
  frame.source_pts = false;
  const GstSegment* segment = gst_sample_get_segment(sample.get());
  const GstClockTime pts = GST_BUFFER_PTS(buffer);
  GstClock* clock = gst_element_get_clock(pipeline_);
  if (clock && segment && segment->format == GST_FORMAT_TIME && GST_CLOCK_TIME_IS_VALID(pts)) {
    const GstClockTime running = gst_segment_to_running_time(segment, GST_FORMAT_TIME, pts);
    const GstClockTime base = gst_element_get_base_time(pipeline_);
    const GstClockTime current = gst_clock_get_time(clock);
    if (GST_CLOCK_TIME_IS_VALID(running) && GST_CLOCK_TIME_IS_VALID(base) &&
        base + running <= current) {
      // Map the pipeline PTS clock to Unix time. PTS is not a hardware exposure timestamp.
      frame.stamp_ns -= static_cast<int64_t>(current - base - running);
      frame.source_pts = true;
    }
  }
  if (clock) gst_object_unref(clock);
  GstVideoInfo info;
  gst_video_info_init(&info);
  if (!gst_video_info_from_caps(&info, gst_sample_get_caps(sample.get())) ||
      GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGB) {
    throw std::runtime_error("Expected RGB video caps");
  }
  GstVideoFrame mapped;
  if (!gst_video_frame_map(&mapped, &info, buffer, GST_MAP_READ)) {
    throw std::runtime_error("Cannot map camera frame");
  }
  frame.width = GST_VIDEO_INFO_WIDTH(&info);
  frame.height = GST_VIDEO_INFO_HEIGHT(&info);
  const size_t row_bytes = static_cast<size_t>(frame.width) * 3;
  frame.rgb.resize(row_bytes * frame.height);
  const auto* pixels = static_cast<const uint8_t*>(GST_VIDEO_FRAME_PLANE_DATA(&mapped, 0));
  const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&mapped, 0);
  for (uint32_t row = 0; row < frame.height; ++row) {
    std::memcpy(frame.rgb.data() + row * row_bytes, pixels + row * stride, row_bytes);
  }
  gst_video_frame_unmap(&mapped);
  return true;
}
}  // namespace gmsl_camera
