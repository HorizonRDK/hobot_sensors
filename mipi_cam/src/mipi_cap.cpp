/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
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
#include "x3_preparam.h"

#define USE_VPS

MipiDevice::MipiDevice()
{
   // this->dev_name = dev_name;
    this->m_fdVideoDev = -1;
    this->buffers = NULL;
    this->n_buffers = 0;
    this->m_curCaptureIdx = -1;
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
	vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;//VIN_OFFLINE_VPS_OFFINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
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
	vin_info->vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;//VIN_OFFLINE_VPS_OFFINE;

	// 单目的使用dev_id 和 pipe_id 都设置成0
	vin_info->dev_id = 0;
	vin_info->pipe_id = 0;
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
	vin_info->pipe_id = 0;
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
	
	ROS_printf("[%s]->0 cap type=%d,sensor=%d.\n",__func__,m_oHardCapability.m_chip_type,m_oHardCapability.m_sensor_list);
	init_param();
	// sdb 生态开发板，使能sensor mclk
	HB_MIPI_EnableSensorClock(0);
	HB_MIPI_EnableSensorClock(1);
	// 编码模块初始化
	//HB_VENC_Module_Init();
	ret = x3_vp_init();
	if (ret) {
		ROS_printf("[%s]->hb_vp_init failed, ret: %d.\n",__func__, ret);
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
#ifdef USE_VPS
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
			ROS_printf("[%s]x3_vin_bind_vps failed, %d.\n",__func__, ret);
			goto vps_bind_err;
		}
	}
#endif
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
guvc_err:
	x3_vp_deinit();
vp_err:
	HB_VENC_Module_Uninit();
	ROS_printf("[%s]->err.\n",__func__);
	return -1;
}

/*
static void *send_yuv_to_vot(void *ptr) {
	tsThread *privThread = (tsThread*)ptr;
	int ret = 0;
	hb_vio_buffer_t vps_out_buf;
	VOT_FRAME_INFO_S pstVotFrame;

	x3_modules_info_t *info = (x3_modules_info_t *)privThread->pvThreadData;
	mThreadSetName(privThread, __func__);
	while(privThread->eState == E_THREAD_RUNNING) {
		memset(&vps_out_buf, 0, sizeof(hb_vio_buffer_t));
		ret = HB_VPS_GetChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf, 300);
		if (ret != 0) {
			ROS_printf("HB_VPS_GetChnFrame error %d!!!\n", ret);
			continue;
		}
		// 把yuv（1080P）数据发给vot显示
		// 数据结构转换
		memset(&pstVotFrame, 0, sizeof(VOT_FRAME_INFO_S));
		pstVotFrame.addr = vps_out_buf.img_addr.addr[0]; // y分量虚拟地址
		pstVotFrame.addr_uv = vps_out_buf.img_addr.addr[1]; // uv分量虚拟地址
		pstVotFrame.size = 1920*1080*3/2; // 帧大小 1920*1088的帧需要强制成1920*1080
		// 发送数据帧到vo模块
		x3_vot_sendframe(&pstVotFrame);
		HB_VPS_ReleaseChnFrame(info->m_vps_infos.m_vps_info[0].m_vps_grp_id,
				3, &vps_out_buf);
	}
	mThreadFinish(privThread);
	return NULL;
}*/

int MipiDevice::init_param(void)
{
	int ret = 0;
	char sensor_name[32] = {0};
	int width = 0, height = 0, fps = 0;
	memset(&m_oX3UsbCam.m_infos, 0, sizeof(m_oX3UsbCam.m_infos));
	// 根据ipc solution的配置设置vin、vps、venc、bpu模块的使能和参数

	ROS_printf("[%s]->begin.\n",__func__);
	// 1. 配置vin
	m_oX3UsbCam.m_infos.m_enable = 1; // 整个数据通路使能
	m_oX3UsbCam.m_infos.m_vin_enable = 1;
	memset(sensor_name, 0, sizeof(sensor_name));
	strcpy(sensor_name, m_oCamInfo.devName);//g_x3_config.usb_cam_solution.pipeline.sensor_name);
	if (strcmp(sensor_name, "IMX415") == 0) {
		ret = mimx415_linear_vin_param_init(&m_oX3UsbCam.m_infos.m_vin_info);
	} else if (strcmp(sensor_name, "F37") == 0) {
		ret = mf37_linear_vin_param_init(&m_oX3UsbCam.m_infos.m_vin_info);
	} else {
		ROS_printf("[%s]->sensor name not found(%s).\n", __func__,sensor_name);
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
	m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
	m_oX3UsbCam.m_infos.m_vps_enable = 1;
	m_oX3UsbCam.m_infos.m_vps_infos.m_group_num = 1;
	// 配置group的输入
	ret |= vps_grp_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0],
			width, height);
	// 以下是配置group的每一个通道的参数
	m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_chn_num = 1;
	ret |= vps_chn_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
			1,m_oCamInfo.width, m_oCamInfo.height, m_oCamInfo.fps);
	/*
	m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_chn_num = 3;
	//这个默认就有 4 层，8 层，分别为原始分辨率的 一半，再一半，第一次传最大size，第二次传最小size
	// chn2 给 usb gadget
	ret |=  vps_chn_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0],
			2, width, height, fps);
	if (width >= 1920 && height >= 1080) {
		// chn 1 给 bpu 运算算法
		ret |= vps_chn_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[1],
				1,1920, 1080, fps);//960,540
		// chn 3 给 hdmi 显示
		ret |= vps_chn_param_init(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[2],
				3, width/4, height/4, fps);
		//m_oX3UsbCam.m_infos.m_vot_enable = 1;
	}*/
#endif
	/*
	// 3. rgn 配置
	m_oX3UsbCam.m_infos.m_rgn_enable = 1;
	ret = x3_rgn_timestamp_param_init(&m_oX3UsbCam.m_infos.m_rgn_info, 0, 0);
	// 4. bpu算法模型
	if (g_x3_config.usb_cam_solution.pipeline.alog_id != 0) {
		m_oX3UsbCam.m_infos.m_bpu_enable = 1;
		m_oX3UsbCam.m_infos.m_bpu_info.m_alog_id = g_x3_config.usb_cam_solution.pipeline.alog_id;
	}
	// vot 配置
	ret |= vot_param_init(&m_oX3UsbCam.m_infos.m_vot_info);
	*/
	ROS_printf("[%s]-> w:h=%d:%d ,fps=%d sucess.\n",__func__,width,height,fps);
	return ret;
}
int MipiDevice::x3_cam_uninit(void) {
	int i = 0;
#ifdef USE_VPS
	if (m_oX3UsbCam.m_infos.m_vin_enable && m_oX3UsbCam.m_infos.m_vps_enable) {
		x3_vin_unbind_vps(m_oX3UsbCam.m_infos.m_vin_info.pipe_id, m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
				m_oX3UsbCam.m_infos.m_vin_info.vin_vps_mode);
	}
#endif
/*
	if (m_oX3UsbCam.m_infos.m_rgn_enable) {
		x3_rgn_uninit(m_oX3UsbCam.m_infos.m_rgn_info.m_rgn_handle,
				&m_oX3UsbCam.m_infos.m_rgn_info.m_rgn_chn);
	}
	if (m_oX3UsbCam.m_infos.m_bpu_enable) {
		x3_bpu_predict_callback_unregister(m_oX3UsbCam.m_infos.m_bpu_info.m_bpu_handle);
		x3_bpu_predict_unint(m_oX3UsbCam.m_infos.m_bpu_info.m_bpu_handle);
	}*/
	if (m_oX3UsbCam.m_infos.m_vps_enable) {
		for (i = 0; i < m_oX3UsbCam.m_infos.m_vps_infos.m_group_num; i++)
			x3_vps_uninit_wrap(&m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[i]);
	}
	if (m_oX3UsbCam.m_infos.m_vin_enable) {
		x3_vin_deinit(&m_oX3UsbCam.m_infos.m_vin_info);
	}
/*
	if (m_oX3UsbCam.m_infos.m_vot_enable)
		x3_vot_deinit();
*/
	x3_vp_deinit();
	HB_VENC_Module_Uninit();
	//print_debug_infos();
	return 0;
}

int MipiDevice::x3_usb_cam_start(void) {
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &m_oX3UsbCam.m_infos;

#ifdef USE_VPS
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
#endif
	if (m_oX3UsbCam.m_infos.m_vin_enable) {
		ret = x3_vin_start(&m_oX3UsbCam.m_infos.m_vin_info);
		if (ret) {
			ROS_printf("x3_vin_start failed, %d", ret);
			return -3003;
		}
	}
/*
	// 启动算法模块
	if (m_oX3UsbCam.m_infos.m_bpu_enable) {
		x3_bpu_predict_start(m_oX3UsbCam.m_infos.m_bpu_info.m_bpu_handle);
		ROS_printf("x3_bpu_predict_start ok!");
	}
	// 启动osd 时间戳线程
	if (m_oX3UsbCam.m_infos.m_rgn_enable) {
		m_oX3UsbCam.m_rgn_thread.pvThreadData = (void*)&m_oX3UsbCam.m_infos.m_rgn_info.m_rgn_handle;
		mThreadStart(x3_rgn_set_timestamp_thread, &m_oX3UsbCam.m_rgn_thread, E_THREAD_JOINABLE);
	}
	if (m_oX3UsbCam.m_infos.m_vot_enable) {
		m_oX3UsbCam.m_vot_thread.pvThreadData = (void*)infos;
		mThreadStart(send_yuv_to_vot, &m_oX3UsbCam.m_vot_thread, E_THREAD_JOINABLE);
	}*/
	print_debug_infos();
	return 0;
}

int MipiDevice::x3_usb_cam_stop(void)
{
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &m_oX3UsbCam.m_infos;
/*
	if (m_oX3UsbCam.m_infos.m_vot_enable) {
		mThreadStop(&m_oX3UsbCam.m_vot_thread);
	}
	if (m_oX3UsbCam.m_infos.m_bpu_enable) {
		x3_bpu_predict_stop(m_oX3UsbCam.m_infos.m_bpu_info.m_bpu_handle);
	}
	// 停止osd线程
	if (m_oX3UsbCam.m_infos.m_rgn_enable) {
		mThreadStop(&m_oX3UsbCam.m_rgn_thread);
	}*/
	if (m_oX3UsbCam.m_infos.m_vin_enable) {
		x3_vin_stop(&m_oX3UsbCam.m_infos.m_vin_info);
	}
	
#ifdef USE_VPS
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
#endif
	return 0;
}

extern "C" int time_cost_ms(struct timeval *start, struct timeval *end);
int MipiDevice::doCapStreamLoop()
{
	return 0;
	struct timeval time_now = { 0 };
	struct timeval time_next = { 0 };
	int size = -1, ret = 0;
	int time_ms = 0;
	struct timeval select_timeout = {0};
	hb_vio_buffer_t vOut;
	char file_name[128] = "/userdata/robot/mipi.yuv";
	int nSave = 0;
	int nPrintLog = 100;
	do
	{
		ret =x3_vin_get_ouput(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,&vOut);
		if (ret < 0) {
			ROS_printf("[doCapStreamLoop]->HB_VIN_GetPipeFrame error =%d !!!\n",ret);
			usleep(10*1000);
			break;
			continue;
		}
		size = vOut.img_addr.stride_size * vOut.img_addr.height;
		if (0==nSave){
			gettimeofday(&time_now, NULL);
			x3_dump_vio_buf_to_nv12(file_name, &vOut);
			gettimeofday(&time_next, NULL);
			int time_cost = time_cost_ms(&time_now, &time_next);
			ROS_printf("[%s]->dumpToFile yuv cost time %d ms,pipe=%d.\n",
				__func__,time_cost,m_oX3UsbCam.m_infos.m_vin_info.pipe_id);
			nSave = 1;
		}
		++nPrintLog;
		if (nPrintLog>100){
			x3_normal_buf_info_print(&vOut);
			ROS_printf("==>[%s]->yuv ret=%d, stride_size(%d) wxh(%d*%d), size %d\n", __func__,ret,
						vOut.img_addr.stride_size,
						vOut.img_addr.width, vOut.img_addr.height, size);
			nPrintLog = 0;
		}		
		ret = x3_vin_output_release(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,&vOut);
		if(ret<0)
			break;
	} while (1);
	
	return 0;
}

int MipiDevice::childStart()
{
	x3_usb_cam_start();
    return 0;
}

int MipiDevice::childStop()
{
    int r;
	if (!m_fdVideoDev)
		return -1;
	x3_usb_cam_stop();
    return 0;
}

int MipiDevice::GetVpsFrame(int nChnID,int *nVOutW,int *nVOutH,void **frame_buf, unsigned int* len)
{
#ifdef USE_VPS
	int size = -1, ret = 0;
	struct timeval select_timeout = {0};
	hb_vio_buffer_t vOut;
	
	int i = 0;
	int stride = 0, width = 0, height = 0;
	int nTry = 0;
	do{
		ret = HB_VPS_GetChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
				nChnID, &vOut, 1000);
		if (ret < 0) {
			if (++nTry>3)
			{
				ROS_printf("[doCapStreamLoop]->HB_VPS_GetChnFrame chn=%d,goupID=%d,error =%d !!!\n",
					nChnID,m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,ret);
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
			//jump over stride - width Y
			for (i = 0; i < height; i++) {
				memcpy(*frame_buf+i*width, vOut.img_addr.addr[0]+i*stride, width);
			}
			//jump over stride - width UV
			for (i = 0; i < height/2; i++) {
				memcpy(*frame_buf+width * height+i*width, vOut.img_addr.addr[1]+i*stride, width);
			}
		}
		// yuv 转成 rgb8
		HB_VPS_ReleaseChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
				nChnID, &vOut);
		break;
	}while(1);
#endif
	return 0;
}
int MipiDevice::GetFrame(void **frame_buf, unsigned int* len)
{
	//uvc_get_frame()
	//不用回调的方式，直接获取视频
	int size = -1, ret = 0;
	struct timeval select_timeout = {0};
	hb_vio_buffer_t vOut;
	
	int i = 0;
	int stride = 0, width = 0, height = 0;
	do{
#ifdef USE_VPS
		ret = HB_VPS_GetChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
				4, &vOut, 1000);
#else
		ret =x3_vin_get_ouput(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,&vOut);
#endif
		if (ret < 0) {
			ROS_printf("[doCapStreamLoop]->HB_VIN_GetPipeFrame error =%d !!!\n",ret);
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
			//jump over stride - width Y
			for (i = 0; i < height; i++) {
				memcpy(*frame_buf+i*width, vOut.img_addr.addr[0]+i*stride, width);
			}
			//jump over stride - width UV
			for (i = 0; i < height/2; i++) {
				memcpy(*frame_buf+width * height+i*width, vOut.img_addr.addr[1]+i*stride, width);
			}
		}
		// yuv 转成 rgb8
#ifdef USE_VPS
		HB_VPS_ReleaseChnFrame(m_oX3UsbCam.m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id,
				4, &vOut);
#else
		ret = x3_vin_output_release(m_oX3UsbCam.m_infos.m_vin_info.pipe_id,&vOut);
#endif
		break;
	}while(1);
    return m_curCaptureIdx;
}
