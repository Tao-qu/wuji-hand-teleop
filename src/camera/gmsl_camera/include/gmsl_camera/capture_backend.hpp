#pragma once

#include <gst/app/gstappsink.h>
#include <cstdint>
#include <string>
#include <vector>

namespace gmsl_camera {
struct Frame {
  uint32_t width = 0;
  uint32_t height = 0;
  int64_t stamp_ns = 0;
  bool source_pts = false;
  std::vector<uint8_t> rgb;
};

class CaptureBackend {
 public:
  CaptureBackend(int sensor_id, int width, int height, int fps);
  ~CaptureBackend();
  CaptureBackend(const CaptureBackend&) = delete;
  CaptureBackend& operator=(const CaptureBackend&) = delete;
  void start();
  bool capture(Frame& frame);

 private:
  void check_bus();
  GstElement* pipeline_ = nullptr;
  GstAppSink* sink_ = nullptr;
};
}  // namespace gmsl_camera
