#ifndef GST_WEBRTC_ADAPTER_H
#define GST_WEBRTC_ADAPTER_H

#include <string>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

class SignalingClient;

class GstWebRTCAdapter {
public:
  // Make Impl publicly visible so C-style callbacks can legally name it
  struct Impl;

  GstWebRTCAdapter(const std::string& signaling_url, int width = 320, int height = 240, int fps = 30);
  ~GstWebRTCAdapter();

  bool Start();                       // initializes GStreamer pipeline and signaling
  void PushFrame(const cv::Mat& bgr_frame);  // push frame into appsrc
  void OnSignalingMessage(const std::string& msg); // handle offer/answer/candidate

private:
  std::unique_ptr<Impl> impl_;
};

#endif // GST_WEBRTC_ADAPTER_H