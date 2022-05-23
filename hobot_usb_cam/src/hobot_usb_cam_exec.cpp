// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "hobot_usb_cam_node.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

using hobot_usb_cam::HobotUSBCamNode;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opt;
  auto node = std::make_shared<HobotUSBCamNode>(opt);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
