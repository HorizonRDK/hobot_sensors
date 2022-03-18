/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#define __STDC_CONSTANT_MACROS
#include "mipi_cam/mipi_cam.hpp"
#ifdef IMAGE_TRANSPORT_PKG_ENABLED
#include "mipi_cam/usb_cam_utils.hpp"
#endif

#include <unistd.h>
#include <errno.h>
#include <malloc.h>
// #include <rclcpp/rclcpp.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <iostream>
// #include <mipi_cam/msg/formats.hpp>

// #include <sensor_msgs/fill_image.h>

#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace mipi_cam
{

MipiCam::MipiCam()
: io_(IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0),
  avframe_camera_size_(0), avframe_rgb_size_(0), image_(NULL),image_pub_(NULL),
  is_capturing_(false) {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}
MipiCam::~MipiCam()
{
  shutdown();
}

#ifdef IMAGE_TRANSPORT_PKG_ENABLED
bool MipiCam::process_image(const void * src, int len, camera_image_t * dest)
{
  // TODO(oal) return bool from all these
  if (pixelformat_ == PIXEL_FORMAT_YUYV) {
    if (monochrome_) {
      // actually format V4L2_PIX_FMT_Y16, but xioctl gets unhappy
      // if you don't use the advertised type (yuyv)
      mono102mono8(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    } else {
      /*yuyv2rgb(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
          */
      NV12_TO_RGB24((unsigned char *)src, (unsigned char*)dest->image, dest->width , dest->height);
      dest->image_size = 3*dest->width*dest->height;
    }
  } else if (pixelformat_ == PIXEL_FORMAT_UYVY) {
    uyvy2rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == PIXEL_FORMAT_RGB24) {
    rgb242rgb(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == PIXEL_FORMAT_GREY) {
    memcpy(
      dest->image,
      const_cast<char *>(reinterpret_cast<const char *>(src)), dest->width * dest->height);
  }

  return true;
}
#endif

bool MipiCam::is_capturing()
{
  return is_capturing_;
}

bool MipiCam::stop_capturing(void)
{
  if (!is_capturing_) {return false;}

  is_capturing_ = false;
  m_pMipiDev->StopStream();
  return true;
}

void MipiCam::GetCaptureHdl(struct TCapFrame *frame, void *user_args)
{
  MipiCam* pThis = (MipiCam*)user_args;
}
bool MipiCam::start_capturing(void)
{
  if (is_capturing_) {return false;}
  int nRet = m_pMipiDev->StartStream(GetCaptureHdl,this);
  is_capturing_ = nRet==0?true:false;
  return true;
}

bool MipiCam::uninit_device(void)
{
  unsigned int i;

  switch (io_) {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
          std::cerr << "error, quitting, TODO throw " << errno << std::endl;
          return false;  // ("munmap");
        }
      }
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }
      break;
  }

  free(buffers_);
  return true;
}

bool MipiCam::init_device(int image_width, int image_height, int framerate)
{
  unsigned int min;
  
  // strcpy(m_oCamInfo.devName, camera_dev_.c_str());  // "F37");
  snprintf(m_oCamInfo.devName, sizeof(m_oCamInfo.devName), "%s", camera_dev_.c_str());
  m_oCamInfo.fps = framerate;
  m_oCamInfo.height = image_height;
  m_oCamInfo.width = image_width;
  int nRet = m_pMipiDev->OpenCamera(&m_oCamInfo);
  ROS_printf("[%s]->cam %s ret=%d.\r\n", __func__, m_oCamInfo.devName, nRet);
  // fd_ = open(camera_dev_.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == nRet) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("mipi_cam"),
       "Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    return false;  // (EXIT_FAILURE);
  }

  return true;
}

bool MipiCam::close_device(void)
{
  if (m_pMipiDev!=NULL)
  {
    m_pMipiDev->StopStream();
    delete m_pMipiDev;
    m_pMipiDev = NULL;
  }
  return true;
}

bool MipiCam::open_device(void)
{
  m_pMipiDev = new MipiDevice();
  if (NULL==m_pMipiDev) {
    // RCLCPP_ERROR_STREAM(
    //   rclcpp::get_logger("mipi_cam"),
    //   "Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    return false;  // (EXIT_FAILURE);
  }
  
  return true;
}
/*
支持格式：
"yuyv"= PIXEL_FORMAT_YUYV
  "uyvy" = PIXEL_FORMAT_UYVY
  "mjpeg" = PIXEL_FORMAT_MJPEG
  "yuvmono10" = PIXEL_FORMAT_YUVMONO10
  "rgb24" = PIXEL_FORMAT_RGB24
  "grey" = PIXEL_FORMAT_GREY
*/
bool MipiCam::start(
  const std::string & dev, const std::string &outFormat, io_method io_method, pixel_format pixel_format,
  int image_width, int image_height, int framerate)
{
  camera_dev_ = dev;
  out_format_ = outFormat;

#ifndef IMAGE_TRANSPORT_PKG_ENABLED
  if (0 != out_format_.compare("nv12")) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
    "Image transport pkg is unable, support nv12 img encoding only!");
    return false;
  }
#endif
  io_ = io_method;
  monochrome_ = false;
  if (dev.find("/video") != dev.npos)
  {
    // 说明找到了，当前不支持 usb  wuwlNG
    return false;
  }
  pixelformat_ = pixel_format;
  if (pixel_format == PIXEL_FORMAT_YUYV) {
  } else if (pixel_format == PIXEL_FORMAT_UYVY) {
  } else if (pixel_format == PIXEL_FORMAT_MJPEG) {
  } else if (pixel_format == PIXEL_FORMAT_YUVMONO10) {
    monochrome_ = true;
  } else if (pixel_format == PIXEL_FORMAT_RGB24) {
  } else if (pixel_format == PIXEL_FORMAT_GREY) {
    monochrome_ = true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"), "Unknown pixel format.");
    return false;  // (EXIT_FAILURE);
  }
  // TODO(oal) throw exceptions instead of return value checking
  if (!open_device()) {
    return false;
  }
  if (!init_device(image_width, image_height, framerate)) {
    return false;
  }
  if (!start_capturing()) {
    return false;
  }
  image_ = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;  // corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_pub_ = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));
  memcpy((void*)image_pub_,(void*)image_,sizeof(camera_image_t));

  image_->image = reinterpret_cast<char *>(calloc(image_->image_size, sizeof(char *)));
  memset(image_->image, 0, image_->image_size * sizeof(char *));
  image_pub_->image = reinterpret_cast<char *>(calloc(image_pub_->image_size, sizeof(char *)));
  memset(image_pub_->image, 0, image_pub_->image_size * sizeof(char *));
  return true;
}

bool MipiCam::shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (image_) {
    free(image_);
  }
  image_ = NULL;
  if (image_pub_) {
    free(image_pub_);
  }
  image_pub_ = NULL;
  return true;
}

void TestSave(char *pFilePath, char *imgData, int nDlen)
{
  FILE *yuvFd = fopen(pFilePath, "w+");
  if (yuvFd) {
    fwrite(imgData, 1, nDlen, yuvFd);
    fclose(yuvFd);
  }
}

bool MipiCam::get_image(
  builtin_interfaces::msg::Time & stamp,
  std::string & encoding, uint32_t & height, uint32_t & width,
  uint32_t & step, std::vector<uint8_t> & data)
{
  if ((image_->width == 0) || (image_->height == 0)) {
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  // m_pMipiDev->GetFrame((void**)&image_->image,(unsigned int *)&image_->image_size);
  if (m_pMipiDev->GetVpsFrame(1, &image_pub_->width, &image_pub_->height, reinterpret_cast<void**>(&image_->image),
    reinterpret_cast<unsigned int*>(&image_->image_size)))
    return false;
  clock_gettime(CLOCK_REALTIME, &time_start);
  stamp.sec = time_start.tv_sec;
  stamp.nanosec = time_start.tv_nsec;
  height = image_pub_->height;
  width = image_pub_->width;
  //这里出来都是 yuv 的
  step = width;
  if (0 == out_format_.compare("nv12")) {
    encoding = "nv12";
    data.resize(image_->image_size);  // step * height);
    memcpy(&data[0], image_->image, data.size());
  } else {
#ifdef IMAGE_TRANSPORT_PKG_ENABLED
    if (monochrome_) {
      encoding = "mono8";
    } else {
      // TODO(oal) aren't there other encoding types?
      encoding = "rgb8";
      step = width * 3;
    }
    // jpeg，png---opencv 转 rgb8
    process_image(image_->image, image_->image_size, image_pub_);
    // TestSave("/userdata/catkin_ws/test.yuv", image_->image, image_->image_size);
    // TestSave("/userdata/catkin_ws/test.rgb", image_pub_->image, image_pub_->image_size);
    // TODO(oal) create an Image here and already have the memory allocated,
    // eliminate this copy
    data.resize(image_pub_->image_size);  // step * height);
    memcpy(&data[0], image_pub_->image, data.size());
#endif
  }
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "[%s]->enc=%s,step=%d,sz=%d,start %ld->laps=%ld ms.\n",
    __func__, encoding.c_str(), step, image_->image_size , msStart, msEnd - msStart);
  return true;
}

void MipiCam::get_formats()  // std::vector<mipi_cam::msg::Format>& formats)
{
}

MipiCam::io_method MipiCam::io_method_from_string(const std::string & str)
{
  if (str == "mmap") {
    return IO_METHOD_MMAP;
  } else if (str == "read") {
    return IO_METHOD_READ;
  } else if (str == "userptr") {
    return IO_METHOD_USERPTR;
  } else {
    return IO_METHOD_UNKNOWN;
  }
}

MipiCam::pixel_format MipiCam::pixel_format_from_string(const std::string & str)
{
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}
#if 0
std::string MipiCam::pixel_format_to_string(__u32 pixelformat)
{
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
    return "yuyv";
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}
#endif
}  // namespace mipi_cam
