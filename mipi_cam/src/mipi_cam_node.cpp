/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "mipi_cam/mipi_cam_node.hpp"

#include <sstream>
// #include <std_srvs/srv/Empty.h>

#include <string>
#include <memory>

#include <stdarg.h>
extern "C" int ROS_printf(char *fmt, ...)
{
  char buf[512] = { 0 };
	va_list args;
	va_start(args, fmt);
  vsprintf(buf, fmt, args);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt, args);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", buf);
	va_end(args);
}

namespace mipi_cam
{

MipiCamNode::MipiCamNode(const rclcpp::NodeOptions & node_options)
:m_bIsInit(0) ,
  Node("mipi_cam", node_options),
  img_(new sensor_msgs::msg::Image()),
  // img_compressed_(new sensor_msgs::msg::CompressedImage()),
  // image_compressed_pub_(std::make_shared<image_transport::Publisher>(
  //  create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10))),
  //  image_transport::create_publisher(this, "image/compressed_image",  rclcpp::QoS{100}.get_rmw_qos_profile()))),
  image_pub_(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, "image_raw", rclcpp::QoS{100}.get_rmw_qos_profile())))//,
  // service_capture_(
  //   this->create_service<std_srvs::srv::SetBool>(
  //     "set_capture",
  //     std::bind(
  //       &MipiCamNode::service_capture,
  //       this,
  //       std::placeholders::_1,
  //       std::placeholders::_2,
  //       std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);  // 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 1080);  // 480);
  this->declare_parameter("image_width", 1920);  // 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("out_format", "rgb8");  // nv12
  this->declare_parameter("video_device", "F37");  // "IMX415");//"F37");//"/dev/video0");
  // image_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);
  video_compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed",5);
  get_params();
  init();  //外部可能会调用了
  ROS_printf("[%s]->mipinode init sucess.\n",__func__);
}

MipiCamNode::~MipiCamNode()
{
  RCLCPP_WARN(this->get_logger(), "shutting down");
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
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_format") {
      out_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      pixel_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
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
  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  cinfo_.reset(new camera_info_manager::CameraInfoManager(this, camera_name_, camera_info_url_));
  // check for default camera info
  if (!cinfo_->isCalibrated()) {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = img_->header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
    RCLCPP_INFO_STREAM(this->get_logger(),"[wuwl]->Calibrated "<< video_device_name_ << " frameID=" << img_->header.frame_id
      << " width="<< image_width_ << " height=" << image_height_);
  }
  
  img_->header.frame_id = frame_id_;
  // img_compressed_->header.frame_id = frame_id_;
  RCLCPP_INFO(
    this->get_logger(), "[MipiCamNode::%s]->Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", __func__,
    camera_name_.c_str(), video_device_name_.c_str(),
    image_width_, image_height_, io_method_name_.c_str(),
    pixel_format_name_.c_str(), framerate_);
  // set the IO method
  MipiCam::io_method io_method = MipiCam::io_method_from_string(io_method_name_);
  if (io_method == MipiCam::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown IO method '%s'", io_method_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // set the pixel format
  MipiCam::pixel_format pixel_format = MipiCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == MipiCam::PIXEL_FORMAT_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown pixel format '%s'", pixel_format_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // ROS_printf("===>[%s]->start cam.\n",__func__);
  // start the camera
  if (false == mipiCam_.start( video_device_name_.c_str(), out_format_name_.c_str(), io_method, pixel_format,
    image_width_, image_height_, framerate_)) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Don't support video dev '%s'", video_device_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  mipiCam_.get_formats();
  // TODO(oal) should this check a little faster than expected frame rate?
  // TODO(oal) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / framerate_;
  timer_ = this->create_wall_timer( std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&MipiCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "starting timer " << period_ms);
  m_bIsInit = 1;
}

bool MipiCamNode::take_and_send_image()
{
  // grab the image
  if (!mipiCam_.get_image(img_->header.stamp, img_->encoding, img_->height, img_->width,
      img_->step, img_->data))
  {
    RCLCPP_ERROR(this->get_logger(), "grab failed");
    return false;
  }
  // INFO(img_->data.size() << " " << img_->width << " " << img_->height << " " << img_->step);
  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
  ci->header = img_->header;
  image_pub_->publish(*img_, *ci);

  // publish compressed
  // cv::Mat matImage_ = cv_bridge::toCvShare(img_, "rgb8")->image;
  /*cv::Mat matImage_(img_->data,true);
  // cv::imwrite("/userdata/2Mat.jpg",matImage_);
  // ROS_printf("[2]->jpg");

  ros_img_compressed_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", matImage_).toCompressedImageMsg();
  video_compressed_publisher_->publish(*ros_img_compressed_);*/
  /*
  // img_compressed_->header = std_msgs::Header();
  img_compressed_->header.stamp = img_->header.stamp;//ros::Time::now();
  img_compressed_->format = "jpeg";
  std::vector<uchar> encodeing;
  cv::imencode(".jpg", img_->data, encodeing);
  encodeing.swap(img_compressed_->data);
  sensor_msgs::msg::CompressedImage compressImg;
  
  //compressed_output_image.data.resize(compressed_buffer_size);
  //memcpy(&compressed_output_image.data[0], compressed_buffer, compressed_buffer_size);
  image_compressed_pub_.publish(compressImg);//*img_compressed_);
  */
  return true;
}

#include "mipi_cam/usb_cam_utils.hpp"
void MipiCamNode::update()
{
  if (mipiCam_.is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    if (!take_and_send_image()) {
      RCLCPP_WARN(this->get_logger(), "USB camera did not respond in time.");
    }
    // auto diff = now() - t0;
    // INFO(diff.nanoseconds() / 1e6 << " " << int(t0.nanoseconds() / 1e9));
    // RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),"[update]->start %ld \n", mipi_cam::GetTickCount());
  }
}
}  // namespace mipi_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mipi_cam::MipiCamNode)
