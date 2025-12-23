#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include "wicom_webrtc/wicom_webrtc_adapter.h"

#include <memory>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wicom_webrtc_streamer_node");
  ros::NodeHandle pnh("~");

  // Allow either full signaling_url or host+port (defaults chosen to avoid 8080 conflict)
  std::string signaling_url;
  std::string signaling_host;
  int signaling_port;

  if (!pnh.getParam("signaling_url", signaling_url)) {
    pnh.param<std::string>("signaling_host", signaling_host, "localhost");
    pnh.param("signaling_port", signaling_port, 8082);
    signaling_url = std::string("ws://") + signaling_host + ":" + std::to_string(signaling_port);
  }

  int width, height, fps;
  pnh.param("width", width, 320);
  pnh.param("height", height, 240);
  pnh.param("fps", fps, 15);

  std::string image_topic;
  int queue_size;
  pnh.param<std::string>("image_topic", image_topic, "/main_camera/image_raw");
  pnh.param("queue_size", queue_size, 1);

  ROS_INFO_STREAM("[wicom_webrtc_streamer_node] signaling_url=" << signaling_url
                  << " image_topic=" << image_topic
                  << " WxH@FPS=" << width << "x" << height << "@" << fps);

  auto adapter = std::make_shared<GstWebRTCAdapter>(signaling_url, width, height, fps);
  if (!adapter->Start()) {
    ROS_ERROR("Failed to start GstWebRTCAdapter");
    return 1;
  }

  image_transport::ImageTransport it(pnh);
  image_transport::TransportHints hints("raw");
  auto cb = [adapter, width, height](const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::Mat frame = cv_ptr->image;
      if (frame.cols != width || frame.rows != height) {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(width, height));
        adapter->PushFrame(resized);
      } else {
        adapter->PushFrame(frame);
      }
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
      ROS_ERROR("Exception in image callback: %s", e.what());
    }
  };

  image_transport::Subscriber sub = it.subscribe(image_topic, queue_size, cb, ros::VoidPtr(), hints);
  ros::spin();
  return 0;
}