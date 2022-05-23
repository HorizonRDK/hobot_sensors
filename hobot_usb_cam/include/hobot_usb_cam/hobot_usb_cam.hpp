// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOT_USB_CAM_HPP_
#define HOBOT_USB_CAM_HPP_
#include <mutex>
#include <chrono>
#include <string>

extern "C" {
  #include <linux/videodev2.h>
}

namespace hobot_usb_cam {
class HobotUSBCam {
 public:
  typedef enum {
    kIO_METHOD_READ,
    kIO_METHOD_MMAP,
    kIO_METHOD_USERPTR
  }IOMethod;
  typedef enum {
    kPIXEL_FORMAT_YUYV,
    kPIXEL_FORMAT_UYVY,
    kPIXEL_FORMAT_MJPEG,
    kPIXEL_FORMAT_YUVMONO10,
    kPIXEL_FORMAT_RGB24,
    kPIXEL_FORMAT_GREY
  }PixelFormat;
  struct CamInformation {
    CamInformation() = default;
    // CamInformation(const CamInformation &cam_information);
    std::string dev;
    IOMethod io;
    PixelFormat pixel_format;
    int32_t image_width;
    int32_t image_height;
    int32_t framerate;
  };
  struct CamBuffer{
    CamBuffer() = default;
    void   *start;
    size_t  length;
    std::chrono::system_clock::time_point time_point;
    struct v4l2_buffer reserved_buffer;
  };
  HobotUSBCam();
  ~HobotUSBCam();
  bool Init(CamInformation &cam_infomation);
  bool Start(void);
  bool Stop(void);
  bool DeInit();
  bool GetFrame(CamBuffer &cam_buffer);
  bool ReleaseFrame(CamBuffer &cam_buffer);

 private:
  int32_t xioctl(int fh, uint32_t request, void *arg);
  void errno_exit(const char *s);
  bool OpenDevice(void);
  bool InitDevice(void);
  bool InitRead(unsigned int buffer_size);
  bool InitMmap(void);
  bool InitUserspace(unsigned int buffer_size);
  bool CloseDevice(void);
  bool ReadFrame(CamBuffer &cam_buffer);


  /* data */
  typedef enum {
    kSTATE_INITIALLED,
    kSTATE_RUNING,
    kSTATE_STOP,
    kSTATE_UNINITIALLED
  }CamState;
  CamState cam_state_;
  std::mutex cam_mutex_;
  CamInformation cam_information_;
  int cam_fd_;
  int buffer_numbers_;
  CamBuffer *buffers_;
};
}  // namespace hobot_usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
