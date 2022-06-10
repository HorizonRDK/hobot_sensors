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

#include "mipi_cam/mipi_cam_node.hpp"

#include <sstream>
// #include <std_srvs/srv/Empty.h>

#include <string>
#include <memory>
#include <vector>
#include <stdarg.h>

extern "C" int ROS_printf(char *fmt, ...)
{
  char buf[512] = { 0 };
	va_list args;
	va_start(args, fmt);
  vsprintf(buf, fmt, args);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", buf);
	va_end(args);
}
#define PUB_BUF_NUM 5
namespace mipi_cam
{

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions & node_options)
:m_bIsInit(0) ,
  Node("mipi_cam", node_options),
  img_(new sensor_msgs::msg::Image())
{
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", PUB_BUF_NUM);
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);  // 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 1080);  // 480);
  this->declare_parameter("image_width", 1920);  // 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("out_format", "bgr8");  // nv12
  this->declare_parameter("video_device", "F37");  // "IMX415");//"F37");
  get_params();
  init();  //外部可能会调用了
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
    "[%s]->mipinode init sucess.\n", __func__);
}

MipiCamNode::~MipiCamNode()
{
  RCLCPP_WARN(rclcpp::get_logger("mipi_node"), "shutting down");
  mipiCam_.shutdown();
}

void MipiCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto & parameter : parameters_client->get_parameters(
      {"camera_name", "camera_info_url", "out_format", "frame_id", "framerate",
        "image_height", "image_width", "io_method", "pixel_format", "video_device"}))
  {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
      "camera_name value: %s", parameter.value_to_string().c_str());
      camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_format") {
      out_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
      "framerate: %f", parameter.as_double());
      framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
        "image_height_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("mipi_node"),
        "image_width_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      pixel_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
        "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

void MipiCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    mipiCam_.start_capturing();
    response->message = "Start Capturing";
  } else {
    mipiCam_.stop_capturing();
    response->message = "Stop Capturing";
  }
}

void MipiCamNode::init()
{
  if(m_bIsInit)
    return;

  // 使能sensor mclk
  std::vector<std::string> sys_cmds{
    "echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart",
    "echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en",
    "echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq",
    "echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en",
    "echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq"
  };
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }

  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
    rclcpp::get_logger("mipi_node"),
    "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  img_->header.frame_id = frame_id_;
  RCLCPP_INFO(
    rclcpp::get_logger("mipi_node"),
    "[MipiCamNode::%s]->Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    __func__,
    camera_name_.c_str(), video_device_name_.c_str(),
    image_width_, image_height_, io_method_name_.c_str(),
    pixel_format_name_.c_str(), framerate_);
  // set the IO method
  MipiCam::io_method io_method = MipiCam::io_method_from_string(io_method_name_);
#ifdef USING_HBMEM
  if (io_method_name_.compare("shared_mem") == 0) {
    // 创建hbmempub
    publisher_hbmem_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        "hbmem_img", PUB_BUF_NUM);
  }
#endif

  // set the pixel format
  MipiCam::pixel_format pixel_format = MipiCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == MipiCam::PIXEL_FORMAT_UNKNOWN) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
    "Unknown pixel format '%s'", pixel_format_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  ROS_printf("===>[%s]->start cam w:h=%d:%d.\n", __func__, image_width_, image_height_);
  // start the camera
  if (false == mipiCam_.start( video_device_name_.c_str(), out_format_name_.c_str(), io_method, pixel_format,
    image_width_, image_height_, framerate_)) {
    RCLCPP_ERROR_ONCE(rclcpp::get_logger("mipi_node"),
    "video dev '%s' config err", video_device_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  mipiCam_.get_formats();
  const int period_ms = 1000.0 / framerate_;
  if (io_method_name_.compare("shared_mem") != 0) {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      std::bind(&MipiCamNode::update, this));
  } else {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      std::bind(&MipiCamNode::hbmem_update, this));
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("mipi_node"), "starting timer " << period_ms);
  m_bIsInit = 1;
}

bool MipiCamNode::take_and_send_image()
{
  // grab the image
  if (!mipiCam_.get_image(img_->header.stamp, img_->encoding, img_->height, img_->width,
      img_->step, img_->data))
  {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "grab failed");
    return false;
  }
  image_pub_->publish(*img_);
  return true;
}
void MipiCamNode::update()
{
  if (mipiCam_.is_capturing()) {
    if (!take_and_send_image()) {
      RCLCPP_WARN(rclcpp::get_logger("mipi_node"),
      "mipi camera did not respond in time.");
    }
  }
}
void MipiCamNode::hbmem_update()
{
#ifdef USING_HBMEM
  if (mipiCam_.is_capturing()) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      if (!mipiCam_.get_image_mem(msg.time_stamp, msg.encoding, msg.height, msg.width,
        msg.step, msg.data, msg.data_size))
      {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "hbmem_update grab img failed");
        return;
      }
      msg.index = mSendIdx++;
      publisher_hbmem_->publish(std::move(loanedMsg));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_node"), "borrow_loaned_message failed");
    }
  }
#endif
}
}  // namespace mipi_cam
