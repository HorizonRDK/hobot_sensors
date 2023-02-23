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

#include "hobot_mipi_node.hpp"

#include <sstream>
#include <stdarg.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

extern "C" int ROS_printf(char* fmt, ...) {
  char buf[512] = {0};
  va_list args;
  va_start(args, fmt);
  vsprintf(buf, fmt, args);
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "%s", buf);
  va_end(args);
}

#define PUB_BUF_NUM 5
namespace mipi_cam {

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions& node_options)
    : m_bIsInit(0),
      Node("mipi_cam", node_options),
      img_(new sensor_msgs::msg::Image()),
      camera_calibration_info_(new sensor_msgs::msg::CameraInfo()) {

  get_params();
  // init();
}

MipiCamNode::~MipiCamNode() {
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  if (mipiCam_ptr_) {
    mipiCam_ptr_->stop();
    mipiCam_ptr_->deinit();
  }
}

void MipiCamNode::get_params() {
  // declare params
  this->declare_parameter("camera_name", "x3pi");
  this->declare_parameter("channel", 0);
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);  // 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);  // 480);
  this->declare_parameter("image_width", 640);   // 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("out_format", "bgr8");   // nv12
  this->declare_parameter("video_device", "F37");  // "IMX415");//"F37");
  this->declare_parameter("camera_calibration_file_path",
                          "/opt/tros/lib/mipi_cam/config/F37_calibration.yaml");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto& parameter :
       parameters_client->get_parameters({"camera_name",
                                          "camera_info_url",
                                          "out_format",
                                          "channel",
                                          "frame_id",
                                          "framerate",
                                          "image_height",
                                          "image_width",
                                          "io_method",
                                          "video_device",
                                          "camera_calibration_file_path"})) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "camera_name value: %s",
                  parameter.value_to_string().c_str());
      nodePare_.camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "channel") {
      nodePare_.channel_ = parameter.as_int();
    } else if (parameter.get_name() == "camera_info_url") {
      nodePare_.camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_format") {
      nodePare_.out_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "framerate: %f",
                  parameter.as_double());
      nodePare_.framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      nodePare_.image_height_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_height_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "image_width") {
      nodePare_.image_width_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "image_width_ value: %s",
                  parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      nodePare_.video_device_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_calibration_file_path") {
      nodePare_.camera_calibration_file_path_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                  "Invalid parameter name: %s",
                  parameter.get_name().c_str());
    }
  }
}

void MipiCamNode::init() {
  if (m_bIsInit)
    return;
  mipiCam_ptr_ = MipiCam::create_mipicam();
  if (!mipiCam_ptr_ || mipiCam_ptr_->init(nodePare_)) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_node"),
              "[%s]->mipinode init failure.\n",
              __func__);
    // rclcpp::shutdown();
    exit(0);
  }
  image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>("image_raw", PUB_BUF_NUM);
  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", PUB_BUF_NUM);

  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
        rclcpp::get_logger("mipi_node"),
        "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  img_->header.frame_id = frame_id_;
  camera_calibration_info_->header.frame_id = frame_id_;
  RCLCPP_INFO(
      rclcpp::get_logger("mipi_node"),
      "[MipiCamNode::%s]->Initing '%s' (%s) at %dx%d via %s at %i FPS",
      __func__,
      nodePare_.camera_name_.c_str(),
      nodePare_.video_device_name_.c_str(),
      nodePare_.image_width_,
      nodePare_.image_height_,
      io_method_name_.c_str(),
      nodePare_.framerate_);
  // set the IO method
#ifdef USING_HBMEM
  if (io_method_name_.compare("shared_mem") == 0) {
    // 创建hbmempub
    publisher_hbmem_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            "hbmem_img", PUB_BUF_NUM);
  }
#endif

  // start the camera
  if (0 != mipiCam_ptr_->start()) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
                      "video dev '%s' start failed!",
                       nodePare_.video_device_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  const int period_ms = 1000.0 / nodePare_.framerate_;
  if (!mipiCam_ptr_->get_cam_calibration(*camera_calibration_info_,
                        nodePare_.camera_calibration_file_path_)) {
    camera_calibration_info_ = nullptr;
    RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
                "get camera calibration parameters failed");
  }
  if (io_method_name_.compare("shared_mem") != 0) {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::update, this));
  } else {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&MipiCamNode::hbmem_update, this));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"),
                     "starting timer " << period_ms);
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
              "[%s]->mipinode init sucess.\n",
              __func__);
  m_bIsInit = 1;
}

bool MipiCamNode::send_calibration(const builtin_interfaces::msg::Time& stamp) {
  if (camera_calibration_info_ != nullptr) {
    camera_calibration_info_->header.stamp = stamp;
    info_pub_->publish(*camera_calibration_info_);
    return true;
  } else {
    return false;
  }
}

void MipiCamNode::update() {
  if (mipiCam_ptr_->is_capturing()) {
    if (!mipiCam_ptr_->get_image(img_->header.stamp,
                            img_->encoding,
                            img_->height,
                            img_->width,
                            img_->step,
                            img_->data)) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "grab failed");
      return;
    }
    image_pub_->publish(*img_);
    if (send_calibration(img_->header.stamp)) {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "publish camera info.\n");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "Unable to publish camera info.\n");
    }
  }
}

void MipiCamNode::hbmem_update() {
#ifdef USING_HBMEM
  if (mipiCam_ptr_->is_capturing()) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!mipiCam_ptr_->get_image_mem(msg.time_stamp,
                                  msg.encoding,
                                  msg.height,
                                  msg.width,
                                  msg.step,
                                  msg.data,
                                  msg.data_size)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_node"),
                     "hbmem_update grab img failed");
        return;
      }
      msg.index = mSendIdx++;
      publisher_hbmem_->publish(std::move(loanedMsg));
      if (send_calibration(msg.time_stamp)) {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"), "publish camera info.\n");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                    "Unable to publish camera info.\n");
      }
    } else {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
                  "borrow_loaned_message failed");
    }
  }
#endif
}
}  // namespace mipi_cam
