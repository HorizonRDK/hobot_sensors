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