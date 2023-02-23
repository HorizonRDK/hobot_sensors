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
#include "hobot_mipi_node.hpp"

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