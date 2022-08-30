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

#ifndef RGBD_NODE_NODE_HPP_
#define RGBD_NODE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

// #include <vector>
#include <memory>
#include <string>

#include "sensor_msgs/image_encodings.hpp"
// #include "sensor_msgs/msg/compressed_image.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/string.hpp>
#include "rgbd_node/rgbd_cam.hpp"

#ifdef USING_HBMEM
#include "hb_mem_mgr.h"
#include "hbm_img_msgs/msg/hbm_msg480_p.hpp"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif
/*
发布：
depth
infra
color
%s/depth/image_rect_raw
%s/depth/camera_info
%s/depth/color/points
%s/infra/image_rect_raw
// %s/infra/camera_info
%s/color/image_rect_raw
// %s/color/camera_info
*/

namespace rgbd_node
{
class RgbdNode : public rclcpp::Node
{
 public:
  explicit RgbdNode(const rclcpp::NodeOptions& node_options, std::string node_name = "rgbd");
  ~RgbdNode();

  void init();
  void get_params();
  // 获取rgb，获取深度图，获取点云，pub

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

 private:
  static void GetCaptureHdl(struct TCapFrame *frame, void *user_args);
  void timer_ros_pub();
  void timer_hbmem_pub();

  // ShyCam rgbdCam_;
  std::shared_ptr<std::thread> m_spThrdPub;
  std::atomic<bool> stop_;
  void exec_loopPub();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDep_pub_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr img_dep_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgInfra_pub_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr img_infra_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgClr_pub_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr img_clr_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depCam_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr img_pcl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr img_pcl_align_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr imgCam_pub_;
  sensor_msgs::msg::CameraInfo::UniquePtr camera_calibration_info_;

 private:
  void pub_align_pcl(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camPublish,
    TTofRgb_PCDClr &pclRgbDepth, struct timespec time_start);
  void pub_ori_pcl(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr camPublish,
    TofDepth_Info &pclDepth, struct timespec time_start);
#ifdef USING_HBMEM
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr pub_hbmem1080P_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg480P>::SharedPtr pub_hbmemdepth_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg480P>::SharedPtr pub_hbmeminfra_;
#endif

 private:
  std::string _sensor_type = "CP3AM";
  std::string _io_mode = "ros";
  std::string frame_id_ = "";
  std::string camera_calibration_file_path_ = "./config/CP3AM_calibration.yaml";
    
  bool _enabled_read_cam_calibration = true;

  int m_bIsInit;
  int clr_w_ = 1920;
  int clr_h_ = 1080;
  int clr_fps_ = 10;
  bool _enable_clr = true;

  int dep_w_ = 224;
  int dep_h_ = 108;
  int dep_fps_ = 10;
  bool _enable_dep = true;
  bool _enable_rgb_pcl = true;
  bool _enable_pcl = true;

  int infra_w_ = 224;
  int infra_h_ = 108;
  int infra_fps_ = 10;
  bool _enable_infra = true;
  int mSendIdx = 0;
};
}  // namespace rgbd_node
#endif  // RGBD_NODE_NODE_HPP_
