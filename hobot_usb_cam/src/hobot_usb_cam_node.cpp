// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "hobot_usb_cam_node.hpp"

#include <fstream>
#include <thread>
#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>

#include "hobot_usb_cam.hpp"

namespace hobot_usb_cam {
HobotUSBCamNode::HobotUSBCamNode(const rclcpp::NodeOptions &ndoe_options)
: Node("hobot_usb_cam", ndoe_options),
  reading_(false),
  read_thread_(nullptr),
  cam_node_state_(kSTATE_UNINITIALLED) {
  this->declare_parameter("framerate", 30);
  this->declare_parameter("frame_id", "default_usb_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "mjpeg");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("zero-copy", "disabled");

  if (GetParams() == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam GetParams() failed\n\n");
    return;
  }

  // This should be done after get_params()
  SetPublisher();
  HobotUSBCam::CamInformation cam_information;
  cam_information.dev = video_device_name_;
  cam_information.framerate = framerate_;
  cam_information.image_height = image_height_;
  cam_information.image_width = image_width_;
  cam_information.io = io_method_name_;
  cam_information.pixel_format = pixel_format_name_;
  if (cam_.Init(cam_information) == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Init failed\n\n");
    return;
  }
  cam_node_state_ = kSTATE_INITIALLED;
  // Update camera params.
  framerate_  = cam_information.framerate;
  image_height_ = cam_information.image_height;
  image_width_ = cam_information.image_width;
  pixel_format_name_ = cam_information.pixel_format;
  if (cam_.Start() == false) {
    RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Start failed\n\n");
    return;
  }
  cam_node_state_ = kSTATE_RUNING;
  reading_ = true;
  read_thread_ = new std::thread(&HobotUSBCamNode::ReadFrame, this);
}

HobotUSBCamNode::~HobotUSBCamNode() {
  if (read_thread_ != nullptr)
    read_thread_->join();
  if (cam_node_state_ == kSTATE_RUNING) {
    if (cam_.Stop() == false) {
      RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Stop failed\n\n");
    }
    cam_node_state_ = kSTATE_STOP;
  }
  if (cam_node_state_ == kSTATE_STOP) {
    if (cam_.DeInit() == false) {
      RCLCPP_ERROR(this->get_logger(), "Hobot USB Cam Dinit failed\n\n");
    }
  }
}

bool HobotUSBCamNode::GetParams() {
  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {"frame_id", "framerate", "image_height", "image_width",
    "io_method", "pixel_format", "video_device"});
  return AssignParams(parameters);
}

bool HobotUSBCamNode::AssignParams(
  const std::vector<rclcpp::Parameter> & parameters) {
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      framerate_ = parameter.as_int();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      std::string io_method_name = parameter.value_to_string();
      if (SetIOMethod(io_method_name) == false)
        return false;
    } else if (parameter.get_name() == "pixel_format") {
      std::string pixel_format_name = parameter.value_to_string();
      if (SetPixelFormat(pixel_format_name) == false)
        return false;
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "zero-copy") {
      zero_copy_enabled_ = parameter.as_bool();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s",
        parameter.get_name().c_str());
    }
  }
  return true;
}

bool HobotUSBCamNode::SetPixelFormat(const std::string &pixel_format_name) {
  if (pixel_format_name == "mjpeg") {
  } else {
    RCLCPP_WARN(this->get_logger(), "Currently only support "
      "mjpeg pixel format. Set pixel format to mjepg");
  }
  pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_MJPEG;
  // Todo : support yuyv uyvy yuvmono10 rgb24 grey
  // if (pixel_format_name == "yuyv") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_YUYV;
  // } else if (pixel_format_name == "uyvy") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_UYVY;
  // } else if (pixel_format_name == "mjpeg") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_MJPEG;
  // } else if (pixel_format_name == "yuvmono10") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_YUVMONO10;
  // } else if (pixel_format_name == "rgb24") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_RGB24;
  // } else if (pixel_format_name == "grey") {
  //   pixel_format_name_ = HobotUSBCam::kPIXEL_FORMAT_GREY;
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "Invalid parameter name: %s",
  //   pixel_format_name.c_str());
  //   return false;
  // }
  return true;
}

bool HobotUSBCamNode::SetIOMethod(const std::string &io_method_name) {
  if (io_method_name == "mmap") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_MMAP;
  } else if (io_method_name == "read") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_READ;
  } else if (io_method_name == "userptr") {
    io_method_name_ = HobotUSBCam::kIO_METHOD_USERPTR;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid parameter name: %s",
    io_method_name.c_str());
    return false;
  }
  return true;
}

void HobotUSBCamNode::SetPublisher() {
  if (zero_copy_enabled_) {
    if (image_width_ == 1920 && image_height_ == 1080) {
      hbmem_image_pub_1080_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>
          ("hbmem_image", 5);
      hbmem_image_pub_540_ = nullptr;
      hbmem_image_pub_480_ = nullptr;
      image_pub_ = nullptr;
    } else if (image_width_ == 960 && image_height_ == 540) {
      hbmem_image_pub_1080_ = nullptr;
      hbmem_image_pub_540_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg540P>
          ("hbmem_image", 5);
      hbmem_image_pub_480_ = nullptr;
      image_pub_ = nullptr;
    } else if (image_width_ == 640 && image_height_ == 480) {
      hbmem_image_pub_1080_ = nullptr;
      hbmem_image_pub_540_ = nullptr;
      hbmem_image_pub_480_ =
        this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg480P>
          ("hbmem_image", 5);
      image_pub_ = nullptr;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid resolution: "
      "width:%d height:%d", image_width_, image_height_);
    }
  } else {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 5);
  }
}

void HobotUSBCamNode::ReadFrame() {
  while (reading_) {
    HobotUSBCam::CamBuffer cam_buffer;
    cam_.GetFrame(cam_buffer);
    switch (pixel_format_name_) {
    case HobotUSBCam::kPIXEL_FORMAT_MJPEG:
      /* code */
      break;
    case HobotUSBCam::kPIXEL_FORMAT_YUYV:
      break;
    case HobotUSBCam::kPIXEL_FORMAT_UYVY:
      break;
    case HobotUSBCam::kPIXEL_FORMAT_YUVMONO10:
      break;
    case HobotUSBCam::kPIXEL_FORMAT_RGB24:
      break;
    case HobotUSBCam::kPIXEL_FORMAT_GREY:
      break;
    default:
      break;
    }
    if (zero_copy_enabled_) {
      if (image_width_ == 1080) {
        auto loanedMsg = hbmem_image_pub_1080_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto& message = loanedMsg.get();
          message.height = 1080;
          message.width = 1920;
          message.step = 1920 * 3;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.data_size = cam_buffer.length;
          message.time_stamp =
            cam_buffer.reserved_buffer.timestamp.tv_sec * 1000000000
            + cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
          hbmem_image_pub_1080_->publish(std::move(loanedMsg));
        }
      } else if (image_width_ == 540) {
        auto loanedMsg = hbmem_image_pub_540_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto& message = loanedMsg.get();
          message.height = 540;
          message.width = 960;
          message.step = 960 * 3;
          message.data_size = cam_buffer.length;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.time_stamp =
            cam_buffer.reserved_buffer.timestamp.tv_sec * 1000000000
            + cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
          hbmem_image_pub_540_->publish(std::move(loanedMsg));
        }
      } else {
        auto loanedMsg = hbmem_image_pub_540_->borrow_loaned_message();
        if (loanedMsg.is_valid()) {
          auto& message = loanedMsg.get();
          message.height = 480;
          message.width = 640;
          message.step = 640 * 3;
          message.data_size = cam_buffer.length;
          memcpy(message.encoding.data(), "jpeg", strlen("jpeg"));
          message.time_stamp =
            cam_buffer.reserved_buffer.timestamp.tv_sec * 1000000000
            + cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
          memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
          hbmem_image_pub_540_->publish(std::move(loanedMsg));
        }
      }
    } else {
      auto message = sensor_msgs::msg::Image();
      size_t size;
      message.header.frame_id = frame_id_;
      message.height = image_height_;
      message.width = image_width_;
      message.step = image_width_ * 3;
      message.header.stamp.sec = cam_buffer.reserved_buffer.timestamp.tv_sec;
      message.header.stamp.nanosec =
        cam_buffer.reserved_buffer.timestamp.tv_usec * 1000;
      if (pixel_format_name_ == HobotUSBCam::kPIXEL_FORMAT_MJPEG) {
        message.encoding = "jpeg";
        size = cam_buffer.length;
        message.data.resize(size);
        memcpy(&message.data[0], cam_buffer.start, cam_buffer.length);
      } else {
        // Todo support other pixel format
        // message.encoding = "rgb8";
        // size = message.step * message.height;
        // message.data.resize(size);
        // memcpy(&message.data[0], );
      }
      image_pub_->publish(message);
    }
    RCLCPP_INFO(this->get_logger(), "publish image %dx%d"
      " encoding:%d size:%d\n", image_width_, image_height_,
      pixel_format_name_, cam_buffer.length);
    cam_.ReleaseFrame(cam_buffer);
  }
}

}  // namespace hobot_usb_cam
