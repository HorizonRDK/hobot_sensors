// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "include/image_subscriber.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "This is image_subscriber example!");

  rclcpp::NodeOptions opt;
  auto node = std::make_shared<ImageSubscriber>(opt);
  RCLCPP_WARN(rclcpp::get_logger("example"), "ImageSubscriber init!");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  RCLCPP_WARN(rclcpp::get_logger("example"), "ImageSubscriber add_node!");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
