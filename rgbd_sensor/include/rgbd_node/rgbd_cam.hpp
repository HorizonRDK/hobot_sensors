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

#ifndef RGBD_MIPI_CAM_HPP_
#define RGBD_MIPI_CAM_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// #include <sensor_msgs/msg/image.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include <string>
#include <vector>
#include <sstream>
#include "rgbd_node/sunny_camera.h"
#include "video_comm.h"

namespace rgbd_node
{

typedef struct _Shy_stream {
    TShyFrame depth_frame;
    EFrameST enDepthStat;       // 深度图0 ,无数据，1，读取数据，2，用完
    TShyFrame clrTof_frame;     // 主码流 1080P
    // TShyFrame clr_frame;     // 640*480
    EFrameST enClrStat;         // rgb是否获取成功
    bool bUse;                  // 是否正在使用
}TShyStreamData;

class ShyCam : public CVideoComm {
 public:
  ShyCam();
  ~ShyCam();

    static ShyCam *GetInstance()
    {
        static ShyCam obj;
        return &obj;
    }

 public:
    // 启动设备交互，定时任务等，命令 1
    int InitVideo();
    // 获取视频数据
    // int StartStream(CapFrame_Callback_t pFrameCB,void *pArg);
    // int StopStream();
    int SetSyncOutput(bool bSync);  // 设置同步输出颜色、深度图
    // 0 不需要判断，则只要大于empty 就可以返回，主要用于保存图片
    bool GetDepthFrame(TShyFrame &imgDepth, int nCheck = 1);
    bool ReleaseDepthFrame();
    bool GetClrFrame(TShyFrame &imgClr, int nCheck = 1);
    bool ReleaseClrFrame();
    bool ReleasePair();
    int GetCurState(int nType);   // 查询当前存储图片使用状态
    int GetUseIdx() { return m_nCurUseIndex; }
    void SetSaveData(bool bSave) { m_bSave = bSave; }   // 查询当前存储图片使用状态
    int CalcTofSync(TTofRgbResult *pOutTofRes);
    bool ReadCalibrationFile(sensor_msgs::msg::CameraInfo& cam_info, const std::string &file_path);
    
    bool is_capturing() { return m_nDevStat == enDEV_START; }

 private:
    virtual int doCapStreamLoop();
    virtual int childStart();
    virtual int childStop();
    // 缓存对象，可以分别配置，如果关闭 ai，就不需要同步，只取 深度图；否则取对应图像
    void release2save();

 private:
    int m_nPrintNum;
    uint32_t m_lStartTM;
    int m_nDepthNum;     // 深度图个数
    int m_nPrintPairNum;
    int m_nPairNum;      // 一对个数

    bool m_bSave;
    // 如果 一个在写，另一个读不到，这个时候，可能要等到 下一个轮，所以，缓冲区放大一倍
    TShyStreamData m_arrRecvStrmData[BUF_DEPTH * 2];
    // 得到可写的索引
    int GetValidUse();
    mutable std::mutex m_mutxUseIdx;
    // 得到可读的索引
    int GetValidSave();

  std::condition_variable m_condClr;
    mutable std::mutex m_mutxClr;
  std::condition_variable m_condDepth;
    mutable std::mutex m_mutxDepth;

    int m_nCurUseIndex;     // 当前正在使用索引
    int m_nCurSaveIdx;
    int m_nBufNum;
    bool m_bSyncOutput;     // 是否同步clr、depth 图保存
    int m_fdVideo;          // 摄像头设备句柄
    void *m_pTofCamHdl;     // tof 摄像头句柄
    void *m_pRgbCamHdl;     // rgb 摄像头句柄
};
}  // namespace rgbd_node

#endif  // RGBD_MIPI_CAM_HPP_
