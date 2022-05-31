/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rgbd_node/rgbd_node.hpp"

using rgbd_node::RgbdNode;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->This is rgbd!");

  rclcpp::NodeOptions opt;
  auto node = std::make_shared<RgbdNode>(opt);
  node->init();
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->rgbd init!");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->rgbd add_node!");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
