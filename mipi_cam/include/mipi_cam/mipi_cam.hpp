/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef MIPI_CAM__MIPI_CAM_HPP_
#define MIPI_CAM__MIPI_CAM_HPP_

#include <asm/types.h>          /* for videodev2.h */

extern "C"
{
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
}

// legacy reasons
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// #include <sensor_msgs/msg/image.h>
#include <string>
#include <vector>
#include <sstream>

#include "mipi_cap.h"

namespace mipi_cam
{
class MipiCam
{
public:
  typedef enum
  {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
    IO_METHOD_UNKNOWN,
  } io_method;

  typedef enum
  {
    PIXEL_FORMAT_YUYV,
    PIXEL_FORMAT_UYVY,
    PIXEL_FORMAT_MJPEG,
    PIXEL_FORMAT_YUVMONO10,
    PIXEL_FORMAT_RGB24,
    PIXEL_FORMAT_GREY,
    PIXEL_FORMAT_UNKNOWN
  } pixel_format;

  MipiCam();
  ~MipiCam();

  // start camera
  bool start(
    const std::string & dev, io_method io, pixel_format pf,
    int image_width, int image_height, int framerate);
  // shutdown camera
  bool shutdown(void);

  // grabs a new image from the camera
  // bool get_image(sensor_msgs::msg::Image:::SharedPtr image);
  bool get_image(
    builtin_interfaces::msg::Time & stamp, 
    std::string & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step, std::vector<uint8_t> & data);

  void get_formats();  // std::vector<usb_cam::msg::Format>& formats);

  static io_method io_method_from_string(const std::string & str);
  static pixel_format pixel_format_from_string(const std::string & str);

  bool stop_capturing(void);
  bool start_capturing(void);
  bool is_capturing();

private:
  // TODO(oal) just store an Image shared_ptr here
  typedef struct
  {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    builtin_interfaces::msg::Time stamp;
    char * image;
    int is_new;
  } camera_image_t;

  struct buffer
  {
    void * start;
    size_t length;
  };


  int init_mjpeg_decoder(int image_width, int image_height);
  bool mjpeg2rgb(char * MJPEG, int len, char * RGB, int NumPixels);
  bool process_image(const void * src, int len, camera_image_t * dest);
  bool uninit_device(void);
  bool init_device(int image_width, int image_height, int framerate);
  bool close_device(void);
  bool open_device(void);

  rclcpp::Clock::SharedPtr clock_;
  std::string camera_dev_;
  unsigned int pixelformat_;
  bool monochrome_;
  io_method io_;
  int fd_;
  buffer * buffers_;
  unsigned int n_buffers_;
  AVFrame * avframe_camera_;
  AVFrame * avframe_rgb_;
  AVCodec * avcodec_;
  AVDictionary * avoptions_;
  AVCodecContext * avcodec_context_;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  camera_image_t * image_pub_;
  camera_image_t * image_;
  bool is_capturing_;
  MipiDevice *m_pMipiDev = NULL;
  TCamInfo m_oCamInfo;
  static void GetCaptureHdl(struct TCapFrame *frame, void *user_args);
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
