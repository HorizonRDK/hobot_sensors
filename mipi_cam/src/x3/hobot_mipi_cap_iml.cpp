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

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include "vio/hb_vio_interface.h"
#include "cam/hb_cam_interface.h"

#include "sensor_msgs/msg/camera_info.hpp"


#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "x3_sdk_wrap.h"
#include "x3_vio_bind.h"
#include "x3_vio_vin.h"
#include "x3_vio_vp.h"
#include "x3_vio_vps.h"
#include "x3_preparam.h"

namespace mipi_cam {

int HobotMipiCapIml::initEnv(std::string sensor) {
  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;

  parse_config(info.sensor_type, info.width, info.height, info.fps);
  vin_info_.pipe_id = info.pipeline_idx;

  reset_sensor(info.sensor_type);

  ret = x3_vp_init();
  if (ret) {
    ROS_printf("[%s]->hb_vp_init failed, ret: %d.\n", __func__, ret);
    goto vp_err;
  }
  // 初始化算法模块
  // 1. 初始化
  // vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
  ret = x3_vin_init(&vin_info_);
  // {  // check分辨率
    // int sensor_w =vin_info.mipi_attr.mipi_host_cfg.width;
    // int sensor_h =vin_info.mipi_attr.mipi_host_cfg.height;
    // int input_w = m_oCamInfo.width;
    // int input_h = m_oCamInfo.height;
    // if (!checkParams(sensor_w, sensor_h, input_w, input_h)) {
    //  x3_vp_deinit();
    //  return -3;
    // }

  if (ret) {
    ROS_printf("x3_vin_init failed: %d!\n", ret);
    if (HB_ERR_VIN_SIF_INIT_FAIL == abs(ret)) {  //重复打开
      ROS_printf("Cannot open '%s'! You may have started mipi_cam repeatedly?",
          info.sensor_type);
    }
    x3_vp_deinit();
    return -2;
  }
  ROS_printf("x3_vin_init ok!\n");
  // 2. 初始化 vps，创建 group
  if (vps_enable_) {
    ret = x3_vps_init_wrap(&vps_infos_.m_vps_info[0]);
    if (ret) {
      ROS_printf("x3_vps_init failed = %d.\n", ret);
      goto vps_err;
    }
    ROS_printf("x3_vps_init_wrap ok!\n");
  }
  // 4 vin bind vps
  if (vin_enable_ && vps_enable_) {
    ret = x3_vin_bind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
    if (ret) {
      ROS_printf("[%s]x3_vin_bind_vps failed, %d.\n", __func__, ret);
      goto vps_bind_err;
    }
  }
  ROS_printf("=====>[wuwl]->x3_ipc_init success.\n");
  std::cout << "HobotMipiCapIml::init,ret:" << ret << std::endl;
  // m_nDevStat = 1;
  return ret;
vps_bind_err:
  if (vps_enable_) {
    x3_vps_uninit_wrap(&vps_infos_.m_vps_info[0]);
  }
vps_err:
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
vin_err:
  x3_vp_deinit();
vp_err:
  ROS_printf("[%s]->err.\n", __func__);
  return -1;
}

int HobotMipiCapIml::deinit() {
  int i = 0;
  ROS_printf("x3_cam_uninit start.\n");
  if (vin_enable_ && vps_enable_) {
    x3_vin_unbind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
  }
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++)
      x3_vps_uninit_wrap(&vps_infos_.m_vps_info[i]);
  }
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
  x3_vp_deinit();
  return 0;
}

int HobotMipiCapIml::start() {
  int i = 0, ret = 0;
  // if (1 == mState) return 0;
  // 使能 vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      ret = x3_vps_start(
          vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        ROS_printf("x3_vps_start failed, %d", ret);
        return -3;
      }
    }
    ROS_printf("x3_vps_start ok!");
  }
  if (vin_enable_) {
    ret = x3_vin_start(&vin_info_);
    if (ret) {
      ROS_printf("x3_vin_start failed, %d", ret);
      return -3003;
    }
  }
  // print_debug_infos();   // disable by wuwl 20220514
  // mState = 1;
  return 0;
}

int HobotMipiCapIml::stop() {
  int i = 0, ret = 0;
  // if (2 == mState) return 0;
  ROS_printf("x3_mipi_cam_stop start.\n");
  if (vin_enable_) {
    x3_vin_stop(&vin_info_);
  }
  ROS_printf("x3_mipi_cam_stop groupNum=%d.\n",
             vps_infos_.m_group_num);
  // stop vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      x3_vps_stop(vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        ROS_printf("x3_vps_stop failed, %d", ret);
        return -1;
      }
    }
  }
  ROS_printf("x3_mipi_cam_stop end.\n");
  // mState = 2;
  return 0;
}

std::vector<std::string> HobotMipiCapIml::list_sensor() {
  std::vector<std::string> device;
  return device;
}
bool HobotMipiCapIml::has_list_sensor() {
  return false;
}

int HobotMipiCapIml::GetFrame(int nChnID, int* nVOutW, int* nVOutH,
        void* frame_buf, unsigned int bufsize, unsigned int* len) {
  int size = -1, ret = 0;
  struct timeval select_timeout = {0};
  hb_vio_buffer_t vOut;

  int i = 0;
  int stride = 0, width = 0, height = 0;
  int nTry = 0;
  do {
    ret = HB_VPS_GetChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        nChnID,
        &vOut,
        1000);
    if (ret < 0) {
      if (++nTry > 3) {
        if (ret == -268696577 || ret == -268696579 ||
            ret == -268696580) {  // group不存在或非法
           ROS_printf(
            "Invalid groupID: %d! Please check if this group has been "
            "closed in unexpected places!",
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        } else if (ret == -268696581 ||
                   -268696587 == ret) {  // 通道未使能或不存在
           ROS_printf(
            "Invalid chn:%d in groupID: %d! Please check if this channel "
            "has been closed in unexpected places!",
            nChnID,
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        }
        ROS_printf(
          "[GetVpsFrame]->HB_VPS_GetChnFrame chn=%d,goupID=%d,error =%d "
          "!!!\n",
          nChnID,
          vps_infos_.m_vps_info[0].m_vps_grp_id,
          ret);
        return -1;
      }
      usleep(10 * 1000);
      continue;
    }
    size = vOut.img_addr.stride_size * vOut.img_addr.height;
    stride = vOut.img_addr.stride_size;
    width = vOut.img_addr.width;
    height = vOut.img_addr.height;
    *nVOutW = width;
    *nVOutH = height;
    *len = width * height * 3 / 2;
    if (bufsize < *len) {
      return -1;
    }
    if (stride == width) {
      memcpy(frame_buf, vOut.img_addr.addr[0], width * height);
      memcpy(frame_buf + width * height,
             vOut.img_addr.addr[1],
             width * height / 2);
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(
            frame_buf + i * width, vOut.img_addr.addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      for (i = 0; i < height / 2; i++) {
        memcpy(frame_buf + width * height + i * width,
               vOut.img_addr.addr[1] + i * stride,
               width);
      }
    }
    // yuv 转成 rgb8
    HB_VPS_ReleaseChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        nChnID,
        &vOut);
    break;
  } while (1);
  return 0;
}

int HobotMipiCapIml::parse_config(std::string sensor_name,
                                   int w, int h, int fps) {
  int ret = 0;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    imx415_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
    f37_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
    gc4663_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
    // enable, sdb3.0
    std::vector<std::string> sys_cmds{
        "echo 111 > /sys/class/gpio/export",
        "echo out > /sys/class/gpio/gpio111/direction",
        "echo 1 > /sys/class/gpio/gpio111/value",
    };
    for (const auto& sys_cmd : sys_cmds) {
      system(sys_cmd.data());
    }
    imx586_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
    gc4c33_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
    imx219_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
    imx477_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
    ov5647_linear_vin_param_init(&vin_info_);
  } else {
    ROS_printf("[%s]->sensor name not found(%s).\n", __func__, sensor_name);
    // m_oX3UsbCam.m_infos.m_vin_enable = 0;
    return -1;
  }
  auto sensor_bus = get_sensor_bus(sensor_name);
  if (sensor_bus != 0xff)
    vin_info_.snsinfo.sensorInfo.bus_num = sensor_bus;

    // 减少ddr带宽使用量
  vin_info_.vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;
  // 2. 根据vin中的分辨率配置vps
  int width = vin_info_.mipi_attr.mipi_host_cfg.width;
  int height = vin_info_.mipi_attr.mipi_host_cfg.height;
  int mipi_fps = vin_info_.mipi_attr.mipi_host_cfg.fps;
  // vin_info_.vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
  vps_enable_ = 1;
  vps_infos_.m_group_num = 1;
  // 配置group的输入
  ret |= vps_grp_param_init(
      &vps_infos_.m_vps_info[0], width, height);
  // 以下是配置group的每一个通道的参数
  vps_infos_.m_vps_info[0].m_chn_num = 1;
  ret |= vps_chn_param_init(
      &vps_infos_.m_vps_info[0].m_vps_chn_attrs[0],
      2, w, h, fps);
  ROS_printf(
      "[%s]-> w:h=%d:%d ,fps=%d sucess.\n", __func__, w, h, fps);
}

bool HobotMipiCapIml::check_pipeline_opened(int pipeline_idx) {
  std::string cfg_info;
  // 若mipi camera已打开，sif_info返回“pipeid”
  std::ifstream sif_info("/sys/devices/platform/soc/a4001000.sif/cfg_info");
  if (sif_info.is_open()) {
    while (getline(sif_info, cfg_info)) {
      std::string pipe_line_info =
            "pipe " + std::to_string(pipeline_idx) + " not inited";
      if (!cfg_info.compare(pipe_line_info)) {
        ROS_printf("mipi camera already in use.\n");
        return false;
      }
    }
  }
  return true;
}

int HobotMipiCapIml::reset_sensor(std::string sensor) {
  std::cout << "HobotMipiCapIml::reset_sensor" << std::endl;
  return 0;
}

int HobotMipiCapIml::get_sensor_bus(std::string &sensor_name) {
  return 0xff;
}

}  // namespace mipi_cam
