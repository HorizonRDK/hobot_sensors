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

#include "guvc/uvc_gadget_api.h"
#include "guvc/uvc_gadget.h"

#define USE_VPS

static int uvc_get_frame(struct uvc_context *ctx, void **buf_to,
		int *buf_len, void **entity, void *userdata);
static void uvc_release_frame(struct uvc_context *ctx, void **entity,
		void *userdata);		
static void uvc_streamon_off(struct uvc_context *ctx, int is_on,
		void *userdata);

static PAYLOAD_TYPE_E fcc_to_video_format(unsigned int fcc) {
	PAYLOAD_TYPE_E format;
	switch (fcc) {
	case V4L2_PIX_FMT_NV12:
		format = PT_NV;
		break;
	case V4L2_PIX_FMT_MJPEG:
		format = PT_MJPEG;
		ROS_printf("format is MJPEG\n");
		break;
	case V4L2_PIX_FMT_H265:
		format = PT_H265;
		break;
	case V4L2_PIX_FMT_H264:
	default:
		format = PT_H264;
		ROS_printf("format is H264\n");
	}
	return format;
}

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
#ifdef USE_USBGUVC
	// 初始化USB Gadget 模块
	ret = x3_guvc_init(&m_oX3UsbCam);
	if (ret) {
		ROS_printf("x3_guvc_init failed");
		goto guvc_err;
	}
#endif
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
	x3_guvc_uninit(&m_oX3UsbCam);
guvc_err:
	x3_vp_deinit();
vp_err:
	HB_VENC_Module_Uninit();
	ROS_printf("[%s]->err.\n",__func__);
	return -1;
}

int MipiDevice::x3_guvc_init(x3_usb_cam_t *x3_usb_cam)
{
	struct uvc_params params;
	char *uvc_devname = NULL;  /* uvc Null, lib will find one */
	char *v4l2_devname = NULL; /* v4l2 Null is Null... */
	int ret = 0;

	/* init uvc user params, just use default params and overlay
	 * some specify params.*/
	uvc_gadget_user_params_init(&params);
#if 0
	// bulk mode
	params.bulk_mode = 1;
	params.h264_quirk = 0;
	params.burst = 9;
#else
	// isoc
	params.bulk_mode = 0; /* use bulk mode  */
	params.h264_quirk = 0;
	// params.burst = 9;
	params.mult = 2;
#endif
	ret = uvc_gadget_init(&x3_usb_cam->uvc_ctx, uvc_devname, v4l2_devname, &params);
	if (ret < 0) {
		ROS_printf("uvc_gadget_init error!");
		return -1;
	}
#ifdef EPTZ
	uvc_set_streamon_handler(x3_usb_cam->uvc_ctx, uvc_streamon_off_eptz, x3_usb_cam);
#else
	uvc_set_streamon_handler(x3_usb_cam->uvc_ctx, uvc_streamon_off, x3_usb_cam);
#endif
	/* prepare/release buffer with video queue */
	uvc_set_prepare_data_handler(x3_usb_cam->uvc_ctx, uvc_get_frame, x3_usb_cam);
	uvc_set_release_data_handler(x3_usb_cam->uvc_ctx, uvc_release_frame, x3_usb_cam);
	ROS_printf("x3_guvc_init ok\n");
	return 0;
}

void MipiDevice::x3_guvc_uninit(x3_usb_cam_t* x3_usb_cam)
{
	if (x3_usb_cam == NULL)
		return;
#ifdef USE_USBGUVC
	uvc_gadget_deinit(x3_usb_cam->uvc_ctx);
#endif
}

int MipiDevice::x3_guvc_start(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
#ifdef USE_USBGUVC
	ret = uvc_gadget_start(x3_usb_cam->uvc_ctx);
	if (ret < 0) {
		ROS_printf("uvc_gadget_start error!");
	}
	ROS_printf("x3_guvc_start ok\n");
#endif
	return ret;
}

int MipiDevice::x3_guvc_stop(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
#ifdef USE_USBGUVC
	ret = uvc_gadget_stop(x3_usb_cam->uvc_ctx);
	if (ret < 0) {
		ROS_printf("x3_guvc_stop error!");
	}
#endif
	return ret;
}

#define ALIGN_8(v) ((v + (8 - 1)) / 8 * 8)
static int x3_uvc_venc_init(x3_usb_cam_t *x3_usb_cam)
{
	int ret = 0;
	PIXEL_FORMAT_E pixFmt = HB_PIXEL_FORMAT_NV12;
	VENC_RC_ATTR_S *pstRcParam;
	VENC_CHN_ATTR_S vencChnAttr;

	int payload_type = x3_usb_cam->m_payload_type;
	int width = x3_usb_cam->m_dst_width;
	int height = x3_usb_cam->m_dst_height;
	int stride = x3_usb_cam->m_dst_stride;
	VENC_CHN venc_chn = x3_usb_cam->m_venc_chn;
	height = ALIGN_8(height);

	// 处理输入对齐问题
	/*if (payload_type == PT_H264) {*/
	ROS_printf("width:%d height:%d payload_type:%d", width, height, payload_type);
	memset(&vencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
	vencChnAttr.stVencAttr.enType = (PAYLOAD_TYPE_E)payload_type;
	vencChnAttr.stVencAttr.u32PicWidth = width;
	vencChnAttr.stVencAttr.u32PicHeight = height;
	vencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
	vencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
	vencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
	vencChnAttr.stVencAttr.bEnableUserPts = HB_TRUE;
	vencChnAttr.stVencAttr.s32BufJoint = 0;
	vencChnAttr.stVencAttr.s32BufJointSize = 8000000;
	vencChnAttr.stVencAttr.enPixelFormat = pixFmt;
	vencChnAttr.stVencAttr.u32BitStreamBufferCount = 3;
	vencChnAttr.stVencAttr.u32FrameBufferCount = 3;
	vencChnAttr.stGopAttr.u32GopPresetIdx = 6;
	vencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
	int size = width * height;
	int streambuf = size & 0xfffff000;
	if (size > 2688 * 1522) {
		vencChnAttr.stVencAttr.vlc_buf_size = 7900*1024;
	} else if (size > 1920 * 1080) {
		vencChnAttr.stVencAttr.vlc_buf_size = 4*1024*1024;
	} else if (size > 1280 * 720) {
		vencChnAttr.stVencAttr.vlc_buf_size = 2100*1024;
	} else if (size > 704 * 576) {
		vencChnAttr.stVencAttr.vlc_buf_size = 2100*1024;
	} else {
		vencChnAttr.stVencAttr.vlc_buf_size = 2048*1024;
	}
	if (payload_type == PT_MJPEG) {
		if (x3_usb_cam->m_dst_stride) {
			vencChnAttr.stVencAttr.u32PicWidth = x3_usb_cam->m_dst_stride;
		}
		vencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
		vencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
		vencChnAttr.stVencAttr.stAttrJpeg.quality_factor = 0;
		vencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
		vencChnAttr.stVencAttr.u32BitStreamBufSize = streambuf;
		vencChnAttr.stVencAttr.stCropCfg.bEnable = HB_TRUE;
		vencChnAttr.stVencAttr.stCropCfg.stRect.s32X = 0;
		vencChnAttr.stVencAttr.stCropCfg.stRect.s32Y = 0;
		vencChnAttr.stVencAttr.stCropCfg.stRect.u32Width = width;
		vencChnAttr.stVencAttr.stCropCfg.stRect.u32Height = height;
	} else if (payload_type == PT_H264) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		vencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
		vencChnAttr.stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
		vencChnAttr.stRcAttr.stH264Vbr.u32IntraQp = 20;
		vencChnAttr.stRcAttr.stH264Vbr.u32IntraPeriod = 20;
		vencChnAttr.stRcAttr.stH264Vbr.u32FrameRate = 30;
		vencChnAttr.stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_UNSPECIFIED;
		vencChnAttr.stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL_UNSPECIFIED;
	}
	ret = HB_VENC_CreateChn(venc_chn, &vencChnAttr);
	if (ret != 0) {
		ROS_printf("HB_VENC_CreateChn %d failed, %d.\n", 0, ret);
		return -1;
	}
	pstRcParam = &(vencChnAttr.stRcAttr);
	if (payload_type == PT_MJPEG) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
		ret = HB_VENC_GetRcParam(venc_chn, pstRcParam);
		if (ret != 0) {
			ROS_printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}
		pstRcParam->stMjpegFixQp.u32FrameRate = 30;
		pstRcParam->stMjpegFixQp.u32QualityFactort = 70;
	} else if (payload_type == PT_H264) {
		vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		ret = HB_VENC_GetRcParam(venc_chn, pstRcParam);
		if (ret != 0) {
			ROS_printf("HB_VENC_GetRcParam failed.\n");
			return -1;
		}
		pstRcParam->stH264Cbr.u32BitRate = 12000;
		pstRcParam->stH264Cbr.u32FrameRate = 30;
		pstRcParam->stH264Cbr.u32IntraPeriod = 60;
		pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
		pstRcParam->stH264Cbr.u32IntraQp = 30;
		pstRcParam->stH264Cbr.u32InitialRcQp = 30;
		pstRcParam->stH264Cbr.bMbLevelRcEnable = HB_FALSE;
		pstRcParam->stH264Cbr.u32MaxIQp = 51;
		pstRcParam->stH264Cbr.u32MinIQp = 10;
		pstRcParam->stH264Cbr.u32MaxPQp = 51;
		pstRcParam->stH264Cbr.u32MinPQp = 10;
		pstRcParam->stH264Cbr.u32MaxBQp = 51;
		pstRcParam->stH264Cbr.u32MinBQp = 10;
		pstRcParam->stH264Cbr.bHvsQpEnable = HB_FALSE;
		pstRcParam->stH264Cbr.s32HvsQpScale = 2;
		pstRcParam->stH264Cbr.u32MaxDeltaQp = 3;
		pstRcParam->stH264Cbr.bQpMapEnable = HB_FALSE;
	}
	ret = HB_VENC_SetChnAttr(venc_chn, &vencChnAttr); // config
	if (ret != 0) {
		ROS_printf("HB_VENC_SetChnAttr failed\n");
		return -1;
	}
	VENC_RECV_PIC_PARAM_S pstRecvParam;
	pstRecvParam.s32RecvPicNum = 0;  // unchangable
	ret = HB_VENC_StartRecvFrame(venc_chn, &pstRecvParam);
	if (ret != 0) {
		ROS_printf("HB_VENC_StartRecvFrame failed\n");
		return -1;
	}
	/*system("cat /sys/kernel/debug/vpu/venc");*/
	return ret;
}

static int x3_uvc_venc_uninit(x3_usb_cam_t* x3_usb_cam)
{
	int ret = 0;
	ret = HB_VENC_StopRecvFrame(x3_usb_cam->m_venc_chn);
	if (ret != 0) {
		ROS_printf("HB_VENC_StopRecvFrame failed\n");
		return -1;
	}
	ret = HB_VENC_DestroyChn(x3_usb_cam->m_venc_chn);
	if (ret != 0) {
		ROS_printf("HB_VENC_DestroyChn failed\n");
		return -1;
	}
	return ret;
}
static int x3_uvc_vps_set_chn_attr(int vps_grp_id, int vps_chn_id, int width, int height)
{
	int ret = 0;
	VPS_CHN_ATTR_S chn_attr;
	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	ret = HB_VPS_GetChnAttr(vps_grp_id, vps_chn_id, &chn_attr);
	if (ret) {
		ROS_printf("HB_VPS_GetChnAttr error, ret:%d\n", ret);
	}
	chn_attr.width = width;
	chn_attr.height = height;
	ret = HB_VPS_SetChnAttr(vps_grp_id, vps_chn_id, &chn_attr);
	if (ret) {
		ROS_printf("HB_VPS_SetChnAttr error, ret:%d\n", ret);
	}
	print_vps_chn_attr(&chn_attr);
	system("cat /sys/devices/platform/soc/a4040000.ipu/info/pipeline0_info");
	return ret;
}

static int x3_uvc_vps_set_crop(int vps_grp_id, int vps_chn_id, int x1, int y1, int x2, int y2)
{
	int ret = 0;
	VPS_CROP_INFO_S cropInfo;

	memset(&cropInfo, 0, sizeof(VPS_CROP_INFO_S));
	ret = HB_VPS_GetChnCrop(vps_grp_id, vps_chn_id, &cropInfo);
	if (ret) {
		ROS_printf("HB_VPS_GetChnCrop error, ret:%d\n", ret);
	}
	cropInfo.en = 1;
	cropInfo.cropRect.x = x1;
	cropInfo.cropRect.y = y1;
	cropInfo.cropRect.width = x2 - x1;
	cropInfo.cropRect.height = y2 - y1;
	ret = HB_VPS_SetChnCrop(vps_grp_id, vps_chn_id, &cropInfo);
	if (ret) {
		ROS_printf("HB_VPS_SetChnCrop error, ret:%d\n", ret);
	}
	return ret;
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
	x3_guvc_uninit(&m_oX3UsbCam);
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
#ifdef USE_USBGUVC
	// 启动usb gadget模块
	x3_guvc_start(&m_oX3UsbCam);
#endif
	print_debug_infos();
	return 0;
}

int MipiDevice::x3_usb_cam_stop(void)
{
	int i = 0, ret = 0;
	x3_modules_info_t * infos = &m_oX3UsbCam.m_infos;
#ifdef USE_USBGUVC
	x3_guvc_stop(&m_oX3UsbCam);
#endif
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

int MipiDevice::x3_usb_cam_param_set(CAM_PARAM_E type, char* val, unsigned int length)
{
	switch(type)
	{
	case CAM_VENC_BITRATE_SET:
		{
			// 只设置主码流的比特率
			x3_venc_set_bitrate(0, *(int *)val);
			break;
		}
	default:
		break;
	}
	return 0;
}

int MipiDevice::x3_usb_cam_param_get(CAM_PARAM_E type, char* val, unsigned int* length)
{
	int ret = 0;
	switch(type)
	{
	case CAM_GET_CHIP_TYPE:
		{
			E_CHIP_TYPE chip_id = x3_get_chip_type();
			sprintf(val, "%s", chip_id == E_CHIP_X3M ? "X3M" : "X3E");
			break;
		}
	default:
		{
			ret= -1;
			break;
		}
	}

	return ret;
}

static int uvc_get_frame(struct uvc_context *ctx, void **buf_to,
		int *buf_len, void **entity, void *userdata) {
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;
#ifdef USE_USBGUVC
	VIDEO_FRAME_S pstFrame;
	int ret = 0;
	static int pts = 0;
	static time_t start = 0;
	static time_t last = 0;
	static int idx = 0;

	if (!ctx || !ctx->udev) {
		ROS_printf("uvc_get_frame: input params is null\n");
		return -EINVAL;
	}

	if (!ctx->udev->is_streaming) // usb host是否在请求数据
		return 0;

	if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
		VIDEO_STREAM_S *pstStream = (VIDEO_STREAM_S *)malloc(sizeof(VIDEO_STREAM_S));
		if (!pstStream) {
			return -ENOMEM;
		};

		memset(pstStream, 0, sizeof(VIDEO_STREAM_S));
		// 获取编码数据流
		ret = HB_VENC_GetStream(x3_usb_cam->m_venc_chn, pstStream, 2000);
		if (ret < 0) {
			ROS_printf("HB_VENC_GetStream error!!!\n");
			return -EINVAL;
		}
		/*ROS_printf("HB_VENC_GetStream succeed, pstStream:%p", pstStream);*/
		*buf_to =  pstStream->pstPack.vir_ptr;
		*buf_len = pstStream->pstPack.size;
		*entity = pstStream;
#if 0
		start = time(NULL);
		idx++;
		if (start > last) {
			ROS_printf("guvc venc fps %d\n", idx);
			last = start;
			idx = 0;
		}
#endif
	} else if (x3_usb_cam->m_payload_type == PT_NV) {
		hb_vio_buffer_t *vps_out_buf = (hb_vio_buffer_t *)malloc(sizeof(hb_vio_buffer_t));
		if (!vps_out_buf) {
			return -ENOMEM;
		};
		int vps_grp_id = x3_usb_cam->m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id;
		int vps_chn_id = x3_usb_cam->m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;
		memset(vps_out_buf, 0, sizeof(hb_vio_buffer_t));

		ret = HB_VPS_GetChnFrame(vps_grp_id, vps_chn_id, vps_out_buf, 2000);
		if (ret) {
			ROS_printf("HB_VPS_GetChnFrame error!!!");
			return -EINVAL;
		}

#if 1 // 内存拷贝的方式
		char *mem_yuv = (char *)malloc(vps_out_buf->img_addr.height * vps_out_buf->img_addr.width * 3 / 2);
		memcpy(mem_yuv, vps_out_buf->img_addr.addr[0],
				vps_out_buf->img_addr.height * vps_out_buf->img_addr.width);
		memcpy(mem_yuv + vps_out_buf->img_addr.height * vps_out_buf->img_addr.width,
				vps_out_buf->img_addr.addr[1],
				vps_out_buf->img_addr.height * vps_out_buf->img_addr.width / 2);

		HB_VPS_ReleaseChnFrame(vps_grp_id, vps_chn_id, vps_out_buf);
		*buf_to =  mem_yuv;
		*buf_len = vps_out_buf->img_addr.height * vps_out_buf->img_addr.width * 3 / 2;
		*entity = mem_yuv;
		free(vps_out_buf);
#else // 让 y 和 uv分量存在连续的内存上，避免内存拷贝
		// 在程序运行前配置 export IPU_YUV_CONSECTIVE=1 让获取出来的y uv 地址保持连续
		// 如果不进行动态重新配置输出分辨率，这样用可以减少一次内存拷贝，但是如果修改了输出分辨率，y 和 uv就不连续了。
		/*ROS_printf("HB_VPS_GetChnFrame width: %d height:%d stride_size: %d",*/
		/*vps_out_buf->img_addr.width, vps_out_buf->img_addr.height, vps_out_buf->img_addr.stride_size);*/

		/*ROS_printf("addr[0]:%p addr[1]:%p", vps_out_buf->img_addr.addr[0], vps_out_buf->img_addr.addr[1]);*/
		*buf_to =  vps_out_buf->img_addr.addr[0];
		*buf_len = vps_out_buf->img_addr.width * vps_out_buf->img_addr.height * 3 / 2;
		*entity = vps_out_buf;
#endif
	}
#endif
	return 0;
}

static void uvc_release_frame(struct uvc_context *ctx, void **entity,
		void *userdata) {
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;
#ifdef USE_USBGUVC
	if (!ctx || !entity || !(*entity)) return;
	if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
		VIDEO_STREAM_S *pstStream = (VIDEO_STREAM_S *)(*entity);
		if (pstStream) {
			int ret = HB_VENC_ReleaseStream(x3_usb_cam->m_venc_chn, pstStream);
			if (ret)
				ROS_printf("HB_VENC_ReleaseStream error");
		}

		free(pstStream);
		*entity = NULL;
	} else if (x3_usb_cam->m_payload_type == PT_NV) {
#if 1 // 内存拷贝方式
		free(*entity);
		*entity = NULL;
#else
		int vps_grp_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_grp_id;
		int vps_chn_id = x3_usb_cam->m_infos->m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;
		hb_vio_buffer_t *vps_out_buf = (hb_vio_buffer_t *)(*entity);
		HB_VPS_ReleaseChnFrame(vps_grp_id, vps_chn_id, vps_out_buf);

		free(vps_out_buf);
		*entity = NULL;
#endif
	}
#endif
}

static void uvc_streamon_off(struct uvc_context *ctx, int is_on,
		void *userdata) {
#ifdef USE_USBGUVC
	struct uvc_device *dev;
	unsigned int fcc = 0;
	x3_usb_cam_t *x3_usb_cam = (x3_usb_cam_t *)userdata;
	dev = ctx->udev;
	fcc = dev->fcc;
	int vps_grp_id = x3_usb_cam->m_infos.m_vps_infos.m_vps_info[0].m_vps_grp_id;
	int vps_chn_id = x3_usb_cam->m_infos.m_vps_infos.m_vps_info[0].m_vps_chn_attrs[0].m_chn_id;

	ROS_printf("uvc_stream on/off value:%d\n", is_on);
	// 不支持在原始图像上再放大
	if(dev->height > x3_usb_cam->m_infos.m_vin_info.mipi_attr.mipi_host_cfg.height) {
		ROS_printf("Scale up the original image is not supported");
		return;
	}
	if (is_on) {
		if (dev->height >= 240) {
			x3_usb_cam->m_dst_width = dev->width;
			x3_usb_cam->m_dst_height = dev->height;
		} else {
			ROS_printf("uvc_stream on/off fatal error, unsupport pixel solution: %d \n",
					dev->height);
			return;
		}
		// 重新设置vps的输出分辨率
		x3_uvc_vps_set_chn_attr(vps_grp_id, vps_chn_id, dev->width, dev->height);
		x3_usb_cam->m_payload_type = fcc_to_video_format(fcc);
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			x3_uvc_venc_init(x3_usb_cam);
			// vps bind venc
			x3_vps_bind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
		}
		ROS_printf("##STREAMON is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	} else {
		if (x3_usb_cam->m_payload_type == PT_H264 || x3_usb_cam->m_payload_type == PT_MJPEG) {
			// vps unbind venc
			x3_vps_unbind_venc(vps_grp_id, vps_chn_id, x3_usb_cam->m_venc_chn);
			x3_uvc_venc_uninit(x3_usb_cam);
		}
		ROS_printf("##STREAMOFF is_on(%d)## %s(%ux%u)\n", is_on,
				fcc_to_string(fcc), dev->width, dev->height);
	}
#endif
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
