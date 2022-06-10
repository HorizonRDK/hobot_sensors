// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
