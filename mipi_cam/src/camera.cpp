/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mipi_cam/mipi_cam_node.hpp"

using mipi_cam::MipiCamNode;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->This is camera!");
  
  rclcpp::NodeOptions opt;
  auto node = std::make_shared<MipiCamNode>(opt);
  node->init();
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode init!");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode add_node!");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

int main_test(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->This is camera Test!");

  rclcpp::NodeOptions opt;
  rclcpp::WallRate  loop_rate(200000);  // ms
  do {
    RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode loop in!");
    auto node = std::make_shared<MipiCamNode>(opt);
    node->init();
    RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode init!");

    // rclcpp::executors::SingleThreadedExecutor exec;
    // exec.add_node(node);
    // RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode add_node!");
    // exec.spin();
    rclcpp::spin_some(node);
    loop_rate.sleep();
    RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode loop end!");
  }while(1);

  RCLCPP_WARN(rclcpp::get_logger("example"), "[wuwl]->MipiCamNode shutdown!");
  rclcpp::shutdown();
  return 0;
}