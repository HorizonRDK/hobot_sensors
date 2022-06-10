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
#include "mipi_cam/mipi_cap.h"

#include "x3_vio_venc.h"
#include "x3_vio_vin.h"
#include "x3_vio_vps.h"
#include "x3_vio_bind.h"
#include "x3_sdk_wrap.h"
#include "x3_vio_vdec.h"
#include "x3_vio_vot.h"
#include "x3_vio_vp.h"
//#include "x3_bpu.h"
#include "x3_utils.h"
//#include "x3_config.h"
#include "sensor_f37_config.h"
#include "sensor_imx415_config.h"
#include "sensor_gc4663_config.h"
#include "x3_preparam.h"

MipiDevice::MipiDevice()
{
   // this->dev_name = dev_name;
    this->m_fdVideoDev = -1;
    this->buffers = NULL;
    this->n_buffers = 0;
    this->m_curCaptureIdx = -1;
    mState = 0;
}
MipiDevice::~MipiDevice()
{
    x3_cam_uninit();
}
int MipiDevice::get_available_pipeid()
{
    // 一种文件保存信息，一种是共享内存
    return 0;
}
/******************************* F37 方案 **********************************/
int MipiDevice::mf37_linear_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_F37_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_F37_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_F37_LINEAR_BASE;
    vin_info->ldcinfo = LDC_ATTR_F37_LINEAR_BASE;
    vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = get_available_pipeid();
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

int MipiDevice::mf37_dol2_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_1LANE_F37_30FPS_10BIT_DOL2_INFO;
    vin_info->mipi_attr = MIPI_1LANE_SENSOR_F37_30FPS_10BIT_DOL2_ATTR;
    vin_info->devinfo = DEV_ATTR_F37_DOL2_BASE;
    vin_info->pipeinfo = PIPE_ATTR_F37_DOL2_BASE;
    vin_info->disinfo = DIS_ATTR_F37_DOL2_BASE;
    vin_info->ldcinfo = LDC_ATTR_F37_DOL2_BASE;
    vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;  // VIN_OFFLINE_VPS_OFFINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = get_available_pipeid();
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}
/******************************** IMX415 4K 方案 ******************************/
int MipiDevice::mimx415_linear_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_4LANE_IMX415_30FPS_10BIT_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_4LANE_SENSOR_IMX415_30FPS_10BIT_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_IMX415_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_IMX415_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_IMX415_BASE;
    vin_info->ldcinfo = LDC_ATTR_IMX415_BASE;
    vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = get_available_pipeid();
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

/******************************** GC4663 方案 ******************************/
int MipiDevice::mgc4663_linear_vin_param_init(x3_vin_info_t* vin_info)
{
    vin_info->snsinfo = SENSOR_GC4663_30FPS_1440P_LINEAR_INFO;
    vin_info->mipi_attr = MIPI_SENSOR_GC4663_30FPS_1440P_LINEAR_ATTR;
    vin_info->devinfo = DEV_ATTR_GC4663_LINEAR_BASE;
    vin_info->pipeinfo = PIPE_ATTR_GC4663_LINEAR_BASE;
    vin_info->disinfo = DIS_ATTR_GC4663_BASE;
    vin_info->ldcinfo = LDC_ATTR_GC4663_BASE;
    vin_info->vin_vps_mode = VIN_SIF_ONLINE_DDR_ISP_DDR_VPS_ONLINE;

    // 单目的使用dev_id 和 pipe_id 都设置成0
    vin_info->dev_id = 0;
    vin_info->pipe_id = get_available_pipeid();
    vin_info->enable_dev_attr_ex = 0;

    return 0;
}

int MipiDevice::OpenCamera(const TCamInfo * pCamInfo)
{
    int ret = 0;
    memcpy(&m_oCamInfo,pCamInfo,sizeof(TCamInfo));
    // 获取芯片型号和接入的sensor型号 ->x3m
    memset(&m_oHardCapability, 0, sizeof(m_oHardCapability));
    x3_get_hard_capability(&m_oHardCapability);

    ROS_printf("[%s]->0 cap type=%d,sensor=%d.\n",
        __func__, m_oHardCapability.m_chip_type, m_oHardCapability.m_sensor_list);
    init_param();
    // sdb 生态开发板，使能sensor mclk
    HB_MIPI_EnableSensorClock(0);
    HB_MIPI_EnableSensorClock(1);

    ret = x3_vp_init();
    if (ret) {
        ROS_printf("[%s]->hb_vp_init failed, ret: %d.\n", __func__, ret);
        goto vp_err;
    }
    // 初始化算法模块
    // 1. 初始化 vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
    if (m_oX3UsbCam.m_infos.m_vin_enable) {
        ret = x3_vin_init(&m_oX3UsbCam.m_infos.m_vin_info);
        if (ret) {
            ROS_printf("x3_vin_init failed, %d", ret);
            goto vin_err;
        }
        ROS_printf("x3_vin_init ok!\n");
    }
    // 2. 初始化 vps，创建 group
    if (m_oX3UsbCam.m_infos.m_vps_enable) {
        ret = x3_vps_init_wrap(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0]);
        if (ret) {
            ROS_printf("x3_vps_init failed = %d.\n", ret);
            goto vps_err;
        }
        ROS_printf("x3_vps_init_wrap ok!\n");
    }
    // 4 vin bind vps
    if (m_oX3UsbCam.m_infos.m_vin_enable && m_oX3UsbCam.m_infos.m_vps_enable) {
        ret = x3_vin_bind_vps(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,
            m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
            m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode);
        if (ret) {
            ROS_printf("[%s]x3_vin_bind_vps failed, %d.\n", __func__, ret);
            goto vps_bind_err;
        }
    }
    ROS_printf("=====>[wuwl]->x3_ipc_init success.\n");
    m_nDevStat = 1;
    return ret;
vps_bind_err:
    if (m_oX3UsbCam.m_infos.m_vps_enable) {
        x3_vps_uninit_wrap(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0]);
    }
vps_err:
    if (m_oX3UsbCam.m_infos.m_vin_enable) {
        x3_vin_deinit(&m_oX3UsbCam.m_infos.m_vin_info);
    }
vin_err:
    x3_vp_deinit();
vp_err:
    ROS_printf("[%s]->err.\n", __func__);
    return -1;
}

#define USE_VPS
int MipiDevice::init_param(void)
{
    int ret = 0;
    char sensor_name[32] = {0};
    int width = 0, height = 0, fps = 0;
    memset(&m_oX3UsbCam.m_infos, 0, sizeof(m_oX3UsbCam.m_infos));
    // 根据ipc solution的配置设置vin、vps、venc、bpu模块的使能和参数

    ROS_printf("[%s]->begin.\n", __func__);
    // 1. 配置vin
    m_oX3UsbCam.m_infos.m_enable = 1;  // 整个数据通路使能
    m_oX3UsbCam.m_infos.m_vin_enable = 1;
    memset(sensor_name, 0, sizeof(sensor_name));
    snprintf(sensor_name, sizeof(sensor_name), "%s", m_oCamInfo.devName);
    if (strcmp(sensor_name, "IMX415") == 0 ||
        strcmp(sensor_name, "imx415") == 0) {
        ret = mimx415_linear_vin_param_init(&m_oX3UsbCam.m_infos.m_vin_info);
    } else if (strcmp(sensor_name, "F37") == 0 ||
               strcmp(sensor_name, "f37") == 0) {
        ret = mf37_linear_vin_param_init(&m_oX3UsbCam.m_infos.m_vin_info);
        int bus_num =
            m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num;
        std::string dev_i2c = "/dev/i2c-" + std::to_string(bus_num);
        if (access(dev_i2c.data(), F_OK) != 0 && bus_num > 0) {
          // 默认配置的i2c无效，自适应i2c号，实现对硬件平台的自适应。
          for (int dev_id = bus_num - 1; dev_id >= 0; dev_id--) {
            dev_i2c = "/dev/i2c-" + std::to_string(dev_id);
            if (access(dev_i2c.data(), F_OK) == 0) {
              m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num =
              dev_id;
              ROS_printf("Adapt bus_num for /dev/i2c- from %d to %d",
              bus_num,
              m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num);
              break;
            }
          }
        }
    } else if (strcmp(sensor_name, "GC4663") == 0 ||
               strcmp(sensor_name, "gc4663") == 0) {
      ret = mgc4663_linear_vin_param_init(&m_oX3UsbCam.m_infos.m_vin_info);
      int bus_num = m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num;
      std::string dev_i2c = "/dev/i2c-" + std::to_string(bus_num);
      if (access(dev_i2c.data(), F_OK) != 0 && bus_num > 0) {
        // 默认配置的i2c无效，自适应i2c号，实现对硬件平台的自适应。
        for (int dev_id = bus_num - 1; dev_id >= 0; dev_id--) {
          dev_i2c = "/dev/i2c-" + std::to_string(dev_id);
          if (access(dev_i2c.data(), F_OK) == 0) {
            m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num = dev_id;
            ROS_printf(
                "Adapt bus_num for /dev/i2c- from %d to %d", bus_num,
                m_oX3UsbCam.m_infos.m_vin_info.snsinfo.sensorInfo.bus_num);
            break;
          }
        }
      }
    } else {
        ROS_printf("[%s]->sensor name not found(%s).\n", __func__, sensor_name);
        m_oX3UsbCam.m_infos.m_vin_enable = 0;
        return -1;
    }
    // 减少ddr带宽使用量
    m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;
    // 2. 根据vin中的分辨率配置vps
    width = m_oX3UsbCam.m_infos.m_vin_info.mipi_attr.mipi_host_cfg.width;
    height = m_oX3UsbCam.m_infos.m_vin_info.mipi_attr.mipi_host_cfg.height;
    fps = m_oX3UsbCam.m_infos.m_vin_info.mipi_attr.mipi_host_cfg.fps;
#ifdef USE_VPS
    // m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
    m_oX3UsbCam.m_infos.m_vps_enable = 1;
    m_oX3UsbCam.m_infos.m_vps_infos.m_group_num = 1;
    // 配置group的输入
    ret |= vps_grp_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0],
            width, height);
    // 以下是配置group的每一个通道的参数
    m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_chn_num = 1;
    ret |= vps_chn_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
            2, m_oCamInfo.width, m_oCamInfo.height, m_oCamInfo.fps);
#endif
    ROS_printf("[%s]-> w:h=%d:%d ,fps=%d sucess.\n", __func__, width, height, fps);
    return ret;
}

int MipiDevice::x3_cam_uninit(void) {
    int i = 0;
    ROS_printf("x3_cam_uninit start.\n");
    if (m_oX3UsbCam.m_infos.m_vin_enable && m_oX3UsbCam.m_infos.m_vps_enable) {
        x3_vin_unbind_vps(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,
            m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
            m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode);
    }
    if (m_oX3UsbCam.m_infos.m_vps_enable) {
        for (i = 0; i < m_oX3UsbCam.m_infos.m_vps_infos.m_group_num; i++)
            x3_vps_uninit_wrap(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[i]);
    }
    if (m_oX3UsbCam.m_infos.m_vin_enable) {
        x3_vin_deinit(&m_oX3UsbCam.m_infos.m_vin_info);
    }
    x3_vp_deinit();
    return 0;
}

int MipiDevice::x3_mipi_cam_start(void) {
    int i = 0, ret = 0;
    x3_modules_info_t * infos = &m_oX3UsbCam.m_infos;
    if (1 == mState)
        return 0;
    // 使能 vps
    if (m_oX3UsbCam.m_infos.m_vps_enable) {
        for (i = 0; i < m_oX3UsbCam.m_infos.m_vps_infos.m_group_num; i++) {
            ret = x3_vps_start(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[i].m_vps_grp_id);
            if (ret) {
                ROS_printf("x3_vps_start failed, %d", ret);
                return -3;
            }
        }
        ROS_printf("x3_vps_start ok!");
    }
    if (m_oX3UsbCam.m_infos.m_vin_enable) {
        ret = x3_vin_start(&m_oX3UsbCam.m_infos.m_vin_info);
        if (ret) {
            ROS_printf("x3_vin_start failed, %d", ret);
            return -3003;
        }
    }
    // print_debug_infos();   // disable by wuwl 20220514
    mState = 1;
    return 0;
}

int MipiDevice::x3_mipi_cam_stop(void)
{
    int i = 0, ret = 0;
    if (2 == mState)
        return 0;
    ROS_printf("x3_mipi_cam_stop start.\n");
    x3_modules_info_t * infos = &m_oX3UsbCam.m_infos;
    if (m_oX3UsbCam.m_infos.m_vin_enable) {
        x3_vin_stop(&m_oX3UsbCam.m_infos.m_vin_info);
    }
    ROS_printf("x3_mipi_cam_stop groupNum=%d.\n", m_oX3UsbCam.m_infos.m_vps_infos.m_group_num);
    // stop vps
    if (m_oX3UsbCam.m_infos.m_vps_enable) {
        for (i = 0; i < m_oX3UsbCam.m_infos.m_vps_infos.m_group_num; i++) {
            x3_vps_stop(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[i].m_vps_grp_id);
            if (ret) {
                ROS_printf("x3_vps_stop failed, %d", ret);
                return -1;
            }
        }
    }
    ROS_printf("x3_mipi_cam_stop end.\n");
    mState = 2;
    return 0;
}

extern "C" int time_cost_ms(struct timeval *start, struct timeval *end);
int MipiDevice::doCapStreamLoop()
{
    return 0;
}

int MipiDevice::childStart()
{
    x3_mipi_cam_start();
    return 0;
}

int MipiDevice::childStop()
{
    int r;
    if (!m_fdVideoDev)
        return -1;
    x3_mipi_cam_stop();
    return 0;
}
// #define TEST_CLR
extern void TestSave(char *pFilePath, char *imgData, int nDlen);
int MipiDevice::GetVpsFrame(int nChnID,int *nVOutW,int *nVOutH,void **frame_buf, unsigned int* len)
{
    int size = -1, ret = 0;
    struct timeval select_timeout = {0};
    hb_vio_buffer_t vOut;

    int i = 0;
    int stride = 0, width = 0, height = 0;
    int nTry = 0;
    do {
        ret = HB_VPS_GetChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
                nChnID, &vOut, 1000);
        if (ret < 0) {
            if (++nTry > 3)
            {
                ROS_printf("[GetVpsFrame]->HB_VPS_GetChnFrame chn=%d,goupID=%d,error =%d !!!\n",
                    nChnID, m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id, ret);
                return -1;
            }
            usleep(10*1000);
            continue;
        }
        size = vOut.img_addr.stride_size * vOut.img_addr.height;
        stride = vOut.img_addr.stride_size;
        width = vOut.img_addr.width;
        height = vOut.img_addr.height;
        *nVOutW = width;
        *nVOutH = height;
        *len = width * height * 3 / 2;
        if (stride == width) {
            memcpy(*frame_buf, vOut.img_addr.addr[0], width * height);
            memcpy(*frame_buf + width * height, vOut.img_addr.addr[1], width * height / 2);
        } else {
            // jump over stride - width Y
            for (i = 0; i < height; i++) {
                memcpy(*frame_buf+i*width, vOut.img_addr.addr[0]+i*stride, width);
            }
            // jump over stride - width UV
            for (i = 0; i < height/2; i++) {
                memcpy(*frame_buf+width * height+i*width, vOut.img_addr.addr[1]+i*stride, width);
            }
        }
        // yuv 转成 rgb8
        HB_VPS_ReleaseChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
                nChnID, &vOut);
        break;
    }while(1);
    return 0;
}
int MipiDevice::GetFrame(void **frame_buf, unsigned int* len)
{
    // uvc_get_frame()
    // 不用回调的方式，直接获取视频
    int size = -1, ret = 0;
    struct timeval select_timeout = {0};
    hb_vio_buffer_t vOut;

    int i = 0;
    int stride = 0, width = 0, height = 0;
    do {
        ret = HB_VPS_GetChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
                4, &vOut, 1000);
        if (ret < 0) {
            ROS_printf("[GetFrame]->HB_VIN_GetPipeFrame error =%d !!!\n", ret);
            usleep(10*1000);
            continue;
        }
        size = vOut.img_addr.stride_size * vOut.img_addr.height;
        stride = vOut.img_addr.stride_size;
        width = vOut.img_addr.width;
        height = vOut.img_addr.height;
        *len = width * height * 3 / 2;
        if (stride == width) {
            memcpy(*frame_buf, vOut.img_addr.addr[0], width * height);
            memcpy(*frame_buf + width * height, vOut.img_addr.addr[1], width * height / 2);
        } else {
            // jump over stride - width Y
            for (i = 0; i < height; i++) {
                memcpy(*frame_buf+i*width, vOut.img_addr.addr[0]+i*stride, width);
            }
            // jump over stride - width UV
            for (i = 0; i < height/2; i++) {
                memcpy(*frame_buf+width * height+i*width, vOut.img_addr.addr[1]+i*stride, width);
            }
        }
        // yuv 转成 rgb8
        HB_VPS_ReleaseChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
                4, &vOut);
        break;
    }while(1);
    return m_curCaptureIdx;
}
