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
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifdef BUILD_HBMEM_MSG
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef IMAGE_SUBSCRIBE_EXAMPLE_INCLUDE_IMAGE_SUBSCRIBER_H_
#define IMAGE_SUBSCRIBE_EXAMPLE_INCLUDE_IMAGE_SUBSCRIBER_H_

using rclcpp::NodeOptions;
using ImgCbType =
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &msg)>;

class ImageSubscriber : public rclcpp::Node {
 public:
  ImageSubscriber(const rclcpp::NodeOptions & node_options = NodeOptions(), ImgCbType sub_cb_fn = nullptr,
  std::string node_name = "img_sub", std::string topic_name = "");
  ~ImageSubscriber();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
    subscription_ = nullptr;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr
    subscription_compressed_ = nullptr;
    
  ImgCbType img_cb_ = nullptr;

  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string topic_name_ = "/image_raw";
  std::string topic_name_compressed_ = "/image_raw/compressed";
  // 默认目录为空，表示不保存
  std::string save_dir_ = "";
  // 默认为空，表示正常sub，hbmem，表示用 hbmem sub
  // std::string io_method_ = "";
#ifdef BUILD_HBMEM_MSG
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      hbmem_subscription_;
#endif

  std::chrono::high_resolution_clock::time_point sub_img_tp_;
  int sub_img_frameCount_ = 0;
  std::mutex frame_stat_mtx_;
  std::chrono::high_resolution_clock::time_point sub_imgraw_tp_;
  int sub_imgraw_frameCount_ = 0;
  std::mutex frame_statraw_mtx_;

  // latency
  // 滑动窗口测方差 ， 20 s ，标准帧率 ,只测试原始图
  std::vector<int> m_vecFps;
  std::vector<int> m_vecLatency;
  // int m_nMinFps = 0;
  // int m_nMaxFps = 0;
  // float m_nVarianceFps = 0.0;

  std::chrono::high_resolution_clock::time_point sub_imghbm_tp_;
  int sub_imghbm_frameCount_ = 0;
  std::mutex frame_stathbm_mtx_;
  std::vector<int> m_vecHbmFps;
  std::vector<int> m_vecHbmLatency;
  // int m_nMinHbmFps = 0;
  // int m_nMaxHbmFps = 0;
  // float m_nVarianceHbmFps = 0.0;

  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void topic_compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  // void hbmem_topic_callback(const hbmem_msgs::msg::SampleMessage::SharedPtr msg) const;
#ifdef BUILD_HBMEM_MSG
  void hbmem_topic_callback(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif
};

#define IMAGE_SUBSCRIBER_H_

#endif  // IMAGE_SUBSCRIBER_H_
