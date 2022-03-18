/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef MIPI_CAM__MIPI_CAM_NODE_HPP_
#define MIPI_CAM__MIPI_CAM_NODE_HPP_
#include "mipi_cam/mipi_cam.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifdef IMAGE_TRANSPORT_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "compressed_image_transport/compressed_publisher.h"
#endif

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/string.hpp>

namespace mipi_cam
{
class MipiCamNode : public rclcpp::Node
{
public:
  MipiCamNode(const rclcpp::NodeOptions & node_options);
  ~MipiCamNode();

  void init();
  void get_params();
  void update();
  bool take_and_send_image();

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  MipiCam mipiCam_;

  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;

#ifdef IMAGE_TRANSPORT_PKG_ENABLED
  std::shared_ptr<image_transport::CameraPublisher> image_pub_;
#else
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
#endif

  sensor_msgs::msg::CompressedImage::SharedPtr ros_img_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr video_compressed_publisher_;

  // parameters
  std::string video_device_name_;
  std::string frame_id_;

  std::string io_method_name_;
  // these parameters all have to be a combination supported by the device
  // Use
  // v4l2-ctl --device=0 --list-formats-ext
  // to discover them,
  // or guvcview
  std::string pixel_format_name_;
  std::string out_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;
  int m_bIsInit;

  std::string camera_name_;
  std::string camera_info_url_;
#ifdef IMAGE_TRANSPORT_PKG_ENABLED
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_capture_;
};
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
