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
#include "rclcpp/rclcpp.hpp"
#include "include/image_subscriber.h"
#include <fstream>
// #include "statistics_tracker.hpp"

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
  if (topic_name_.find("hbmem_img") == std::string::npos) {
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
  } else {
#ifdef BUILD_HBMEM_MSG
    hbmem_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            topic_name_, 10,
            std::bind(&ImageSubscriber::hbmem_topic_callback, this,
                      std::placeholders::_1));
    RCLCPP_WARN(rclcpp::get_logger("ImageSubscriber"),
      "Create hbmem_subscription with topic_name: %s, sub = %p", topic_name_.c_str(), hbmem_subscription_);
#endif
  }
}

ImageSubscriber::~ImageSubscriber() {}

#include <sys/times.h>
// 返回ms
int32_t tool_calc_time_laps(struct timespec &time_start, struct timespec &time_end)
{
  int32_t nRetMs = 0;
  if (time_end.tv_nsec < time_start.tv_nsec)
  {
    nRetMs = (time_end.tv_sec - time_start.tv_sec - 1) * 1000 +
     (1000000000 + time_end.tv_nsec - time_start.tv_nsec) / 1000000;
  } else {
    nRetMs = (time_end.tv_sec - time_start.tv_sec) * 1000 + (time_end.tv_nsec - time_start.tv_nsec) / 1000000;
  }
  return nRetMs;
}

void ImageSubscriber::topic_compressed_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) {
  // RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv compressed img");
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
  struct timespec time_now = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = img_msg->header.stamp.nanosec;
  time_in.tv_sec = img_msg->header.stamp.sec;

  std::stringstream ss;
  ss << "Recv compressed img: " << img_msg->format
  // << ", w: " << img_msg->width
  // << ", h: " << img_msg->height
  << ", stamp: " << img_msg->header.stamp.sec
  << "." << img_msg->header.stamp.nanosec
  << ", tmlaps(ms): " << tool_calc_time_laps(time_in, time_now)
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

void TestSave(char *pFilePath, char *imgData, int nDlen)
{
  FILE *yuvFd = fopen(pFilePath, "w+");
  if (yuvFd) {
    fwrite(imgData, 1, nDlen, yuvFd);
    fclose(yuvFd);
  }
}
// static int s_nSave = 0;
#ifdef BUILD_HBMEM_MSG
void ImageSubscriber::hbmem_topic_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  struct timespec time_now = {0, 0}, time_in = {0, 0};
  int32_t nLapsFrm = 0;
  clock_gettime(CLOCK_REALTIME, &time_now);
  // uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  time_in.tv_nsec = msg->time_stamp.nanosec;
  time_in.tv_sec = msg->time_stamp.sec;

  // nLapsFrm = mNow - msg->time_stamp;
  // m_vecHbmLatency.push_back(nLapsFrm);
  std::stringstream ss;
  ss << "Recv raw img: " << msg->encoding.data()
  << ", w: " << msg->width
  << ", h: " << msg->height
  << ", tmlaps(ms): " << tool_calc_time_laps(time_in, time_now)
  << ", data size: " << msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("hbmem_img_sub"), "%s", ss.str().c_str());
  /* if (0 == s_nSave) {
    TestSave("/userdata/test.rgb", (char*)msg->data.data(), msg->data.size());
    s_nSave = 1;
  }
  */
  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stathbm_mtx_);
    sub_imghbm_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imghbm_tp_).count();
    if (interval >= 1000) {
      /*if (sub_imghbm_frameCount_ > 1)
        m_vecHbmFps.push_back(sub_imghbm_frameCount_);
      // 滑动窗口，计算方差
      int nSize = m_vecHbmFps.size();
      if (nSize > 40) {
        std::stringstream ss;
        performance_test::StatisticsTracker statisHbmFps(m_vecHbmFps);
        performance_test::StatisticsTracker statisHbmLatency(m_vecHbmLatency);
        m_vecHbmLatency.clear();
        m_vecHbmFps.clear();
        ss << "Recv hbmImg: " << msg->encoding.data()
        << ", w: " << msg->width
        << ", h: " << msg->height
        << ", minfps: " << statisHbmFps.min()
        << ", maxfps: " << statisHbmFps.max()
        << ", meadfps: " << statisHbmFps.mean()
        << ", variancefps: " << statisHbmFps.variance()
        << ", minLatency: " << statisHbmFps.min()
        << ", maxLatency: " << statisHbmFps.max()
        << ", meadLatency: " << statisHbmFps.mean()
        << ", varianceLatency: " << statisHbmFps.variance()
        << ", data size: " << msg->data_size;
        RCLCPP_INFO(rclcpp::get_logger("img_sub"), "%s", ss.str().c_str());
      }*/
      sub_imghbm_frameCount_ = 0;
      sub_imghbm_tp_ = std::chrono::system_clock::now();
    }
  }
}
#endif

void ImageSubscriber::topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  // RCLCPP_INFO(rclcpp::get_logger("img_sub"), "Recv img");
  // todo
  /* // raw 过大，不保存
  if (save_dir_.length() > 0) {
    std::string fname = save_dir_ + "/raw_img_" +
      std::to_string(msg->header.stamp.sec) + "." +
      std::to_string(msg->header.stamp.nanosec) + msg->encoding;
    std::ofstream ofs(fname);
    ofs.write(reinterpret_cast<const char*>(msg->data.data()),
      msg->data.size());
  }*/
  std::stringstream ss;
  struct timespec time_now = {0, 0}, time_in = {0, 0};
  int32_t nLapsFrm = 0;
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->header.stamp.nanosec;
  time_in.tv_sec = msg->header.stamp.sec;
  nLapsFrm = tool_calc_time_laps(time_in, time_now);
  ss << "Recv raw img: " << msg->encoding
  << ", w: " << msg->width
  << ", h: " << msg->height
  << ", stamp: " << msg->header.stamp.sec
  << "." << msg->header.stamp.nanosec
  << ", tmlaps(ms): " << nLapsFrm
  << ", data size: " << msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("img_sub"), "%s", ss.str().c_str());
  // m_vecLatency.push_back(nLapsFrm);

  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_statraw_mtx_);
    sub_imgraw_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imgraw_tp_).count();
    if (interval >= 1000) {
      /*if (sub_imgraw_frameCount_ > 1)
        m_vecFps.push_back(sub_imgraw_frameCount_);
      // 滑动窗口，计算方差
      int nSize = m_vecFps.size();
      if (nSize > 40) {
        performance_test::StatisticsTracker statisFps(m_vecFps);
        performance_test::StatisticsTracker statisLatency(m_vecLatency);
        m_vecLatency.clear();
        m_vecFps.clear();
        std::stringstream ss;
        ss << "Recv rawImg: " << msg->encoding
        << ", w: " << msg->width
        << ", h: " << msg->height
        << ", minfps: " << statisFps.min()
        << ", maxfps: " << statisFps.max()
        << ", meadfps: " << statisFps.mean()
        << ", variancefps: " << statisFps.variance()
        << ", minLatency: " << statisFps.min()
        << ", maxLatency: " << statisFps.max()
        << ", meadLatency: " << statisFps.mean()
        << ", varianceLatency: " << statisFps.variance()
        << ", data size: " << msg->data.size();
        RCLCPP_INFO(rclcpp::get_logger("img_sub"), "%s", ss.str().c_str());
      }*/
      sub_imgraw_frameCount_ = 0;
      sub_imgraw_tp_ = std::chrono::system_clock::now();
    }
  }

  if (img_cb_) {
    img_cb_(msg);
  }
  return;
}
