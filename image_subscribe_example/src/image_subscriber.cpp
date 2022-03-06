// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "include/image_subscriber.h"
#include <fstream>

ImageSubscriber::ImageSubscriber(const rclcpp::NodeOptions& node_options, ImgCbType sub_cb_fn,
  std::string node_name, std::string topic_name)
    : Node(node_name, node_options), img_cb_(sub_cb_fn) {
  this->declare_parameter("sub_img_topic", topic_name_);
  if (this->get_parameter("sub_img_topic", topic_name_)) {
    RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
     "Update sub_img_topic with topic_name: %s", topic_name_.c_str());
  }
  this->declare_parameter("save_dir", save_dir_);
  if (this->get_parameter("save_dir", save_dir_)) {
    RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
     "Update save_dir: %s", save_dir_.c_str());
  }

  if (!topic_name.empty()) {
    topic_name_ = topic_name;
  }
  RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
  "Create subscription with topic_name: %s", topic_name_.c_str());
  if (topic_name_.compare(topic_name_compressed_) != 0) {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name_, 10,
        std::bind(&ImageSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

  subscription_compressed_ =
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name_compressed_, 10,
      std::bind(&ImageSubscriber::topic_compressed_callback, this,
                std::placeholders::_1));
}

ImageSubscriber::~ImageSubscriber() {}

void ImageSubscriber::topic_compressed_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) {
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv compressed img");
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub compressed img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  std::stringstream ss;
  ss << "Recved compressed img format: " << img_msg->format
  << ", frame_id: " << img_msg->header.frame_id
  << ", stamp: " << img_msg->header.stamp.sec
  << "." << img_msg->header.stamp.nanosec
  << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "%s", ss.str().c_str());

  // dump recved img msg
  if (save_dir_.length() > 0) {
  std::string fname = save_dir_ + "/compressed_img_" +
    std::to_string(img_msg->header.stamp.sec) + "." +
    std::to_string(img_msg->header.stamp.nanosec) + ".jpg";
  std::ofstream ofs(fname);
  ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
    img_msg->data.size());
  }
}

void ImageSubscriber::topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv img");
  // todo
  if (save_dir_.length() > 0) {
    std::string fname = save_dir_ + "/compressed_img_" +
      std::to_string(msg->header.stamp.sec) + "." +
      std::to_string(msg->header.stamp.nanosec) + msg->encoding;
    std::ofstream ofs(fname);
    ofs.write(reinterpret_cast<const char*>(msg->data.data()),
      msg->data.size());
  }
  return;
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    sub_img_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - sub_img_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("img_sub"),
      "Sub img fps = %d", sub_img_frameCount_);
      sub_img_frameCount_ = 0;
      sub_img_tp_ = std::chrono::system_clock::now();
    }
  }

  if (img_cb_) {
    img_cb_(msg);
  }
}
