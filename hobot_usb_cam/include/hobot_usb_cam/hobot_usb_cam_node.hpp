// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOT_USB_CAM_NODE_HPP_
#define HOBOT_USB_CAM_NODE_HPP_

#include <vector>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "hobot_usb_cam.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg540_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg480_p.hpp"


namespace hobot_usb_cam {
class HobotUSBCamNode : public rclcpp::Node {
 public:
  explicit HobotUSBCamNode(const rclcpp::NodeOptions &ndoe_options);
  ~HobotUSBCamNode();

 private:
  typedef enum {
    kSTATE_INITIALLED,
    kSTATE_RUNING,
    kSTATE_STOP,
    kSTATE_UNINITIALLED
  }CamNodeState;
  bool GetParams(void);
  bool AssignParams(const std::vector<rclcpp::Parameter> & parameters);
  bool SetPixelFormat(const std::string &pixel_format_name);
  bool SetIOMethod(const std::string &io_method_name);
  void SetPublisher(void);
  void ReadFrame(void);
  HobotUSBCam cam_;
  std::string video_device_name_;
  std::string frame_id_;
  HobotUSBCam::IOMethod io_method_name_;
  HobotUSBCam::PixelFormat pixel_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;
  bool zero_copy_enabled_;
  bool reading_;
  std::thread *read_thread_;
  CamNodeState cam_node_state_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
    hbmem_image_pub_1080_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg540P>::SharedPtr
    hbmem_image_pub_540_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg480P>::SharedPtr
    hbmem_image_pub_480_;
};
}  // namespace hobot_usb_cam

#endif  // HOBOT_USB_CAM_NODE_HPP_
