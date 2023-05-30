#include <stddef.h>
#include <sys/types.h>
#include "hb_vio_interface.h"
#include "horizon_mpp.h"
#include "hb_venc.h"
#include "hb_vp_api.h"

#include "camera_control.h"
#include "camera_hb_cfg.h"


#define BIT2CHN(chns, chn) (chns & (1 << chn))


int need_ipu = 63;
int minQP = 0;
int maxQP = 0;
unsigned char pu8YuvBuffer[RGB_WIDTH*RGB_HEIGHT*2];
static bool mipi_reset_flag = false;


static int hv_vp_init()
{
	int32_t s32Ret = 0;
	VP_CONFIG_S struVpConf;

	memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
	struVpConf.u32MaxPoolCnt = 32;
	HB_VP_SetConfig(&struVpConf);
	
	s32Ret = HB_VP_Init();
	if (s32Ret != 0) {
		printf("vp_init fail s32Ret = %d !\n", s32Ret);
		return -1;
	}

	return 0;
}


static int hv_vp_deinit()
{
	int32_t s32Ret = 0;
	
	s32Ret = HB_VP_Exit();
	if (s32Ret == 0) 
		printf("vp exit ok!\n");

	return s32Ret;
}


static void print_sensor_info(MIPI_SENSOR_INFO_S *snsinfo)
{
	printf("bus_num %d\n", snsinfo->sensorInfo.bus_num);
	printf("bus_type %d\n", snsinfo->sensorInfo.bus_type);
	printf("sensor_name %s\n", snsinfo->sensorInfo.sensor_name);
	printf("reg_width %d\n", snsinfo->sensorInfo.reg_width);
	printf("sensor_mode %d\n", snsinfo->sensorInfo.sensor_mode);
	printf("sensor_addr 0x%x\n", snsinfo->sensorInfo.sensor_addr);
	printf("serial_addr 0x%x\n", snsinfo->sensorInfo.serial_addr);
	printf("resolution %d\n", snsinfo->sensorInfo.resolution);

	return;
}


/**
 * @brief		hb_enable_sensor_clk
 * @note		
 * @param[in]	
 * @return		0: success,	<0: failed
 */
static int hb_enable_sensor_clk(uint32_t mipiIdx)
{
	int iRet = 0;

	int board_type = GetBoardType();
	if (5 <= GetBoardType() || 9 >= GetBoardType()) {
		// x3pi两个sensor使用的同一个reset管脚，只需要复位一次
		if (!mipi_reset_flag) {
			(void)system("echo 19 > /sys/class/gpio/export");
			(void)system("echo out > /sys/class/gpio/gpio19/direction");
			(void)system("echo 0 > /sys/class/gpio/gpio19/value");
			(void)system("sleep 0.2");
			(void)system("echo 1 > /sys/class/gpio/gpio19/value");
			(void)system("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
			(void)system("echo 1 > /sys/class/vps/mipi_host2/param/stop_check_instart");
			mipi_reset_flag = true;
		}
	} else if (4 == board_type) {
		/* reset使能*/
		if (1 == mipiIdx) {
			(void)system("echo 118 > /sys/class/gpio/export");
			(void)system("echo out > /sys/class/gpio/gpio118/direction");
			(void)system("echo 0 > /sys/class/gpio/gpio118/value");
			(void)system("sleep 0.2");
			(void)system("echo 1 > /sys/class/gpio/gpio118/value");
			(void)system("echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart");
		} else if (0 == mipiIdx) {
			(void)system("echo 119 > /sys/class/gpio/export");
			(void)system("echo out > /sys/class/gpio/gpio119/direction");
			(void)system("echo 0 > /sys/class/gpio/gpio119/value");
			(void)system("sleep 0.2");
			(void)system("echo 1 > /sys/class/gpio/gpio119/value");
			(void)system("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
		}
	} else if (3 == board_type) {
		/* reset使能*/
		if (1 == mipiIdx) {
			(void)system("echo 111 > /sys/class/gpio/export");
			(void)system("echo out > /sys/class/gpio/gpio111/direction");
			(void)system("echo 0 > /sys/class/gpio/gpio111/value");
			(void)system("sleep 0.2");
			(void)system("echo 1 > /sys/class/gpio/gpio111/value");
			(void)system("echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart");
		} else if (0 == mipiIdx) {
			(void)system("echo 119 > /sys/class/gpio/export");
			(void)system("echo out > /sys/class/gpio/gpio119/direction");
			(void)system("echo 0 > /sys/class/gpio/gpio119/value");
			(void)system("sleep 0.2");
			(void)system("echo 1 > /sys/class/gpio/gpio119/value");
			(void)system("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
		}
	} else {
		printf("[%s] Unsupported board type!\n", __func__);
		return -1;
	}
    
	iRet = HB_MIPI_SetSensorClock(mipiIdx, 24000000);
	if (iRet)
	{
		printf("[%s] HB_MIPI_EnableSensorClock failed, iRet = 0x%x\n", __func__, iRet);
		return iRet;
	}

	/* 使能 mclk */
	iRet = HB_MIPI_EnableSensorClock(mipiIdx);
	if (iRet)
	{
		printf("[%s] HB_MIPI_EnableSensorClock failed, iRet = 0x%x\n", __func__, iRet);
		return iRet;
	}

	return 0;
}


static void dis_crop_set(uint32_t pipe_id, uint32_t event, VIN_DIS_MV_INFO_S *data,
			 void *userdata)
{
	printf("dis_crop_set callback come in\n");
	printf("data gmvX %d\n", data->gmvX);
	printf("data gmvY %d\n", data->gmvY);
	printf("data xUpdate %d\n", data->xUpdate);
	printf("data yUpdate %d\n", data->yUpdate);
	return;
}

/**
 * @brief			horizon_vin_vps_init
 * @note		
 * @param[in]		
 * @param[in-out]	pstCamHandle
 * @return			0: success,	<0: failed
 */
static int hb_vin_vps_init(HB_VIN_VPSS_INIT_S *pstHbVinVpssInit, int iNeedIsp)
{
	int iRet = 0;

	VIN_DIS_ATTR_S *disinfo = NULL;
	VIN_LDC_ATTR_S * ldcinfo = NULL;
	VIN_DEV_ATTR_EX_S *devexinfo = NULL;
	VIN_DIS_CALLBACK_S pstDISCallback;
	pstDISCallback.VIN_DIS_DATA_CB = dis_crop_set;

	VPS_GRP_ATTR_S grp_attr;	
	VPS_CHN_ATTR_S chn_attr;	
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	if (!pstHbVinVpssInit)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}
	
	/* 设置在线模式 */
	printf("pipeId[%d], mipiIdx[%d], vin_vps_mode[%u]\n", pstHbVinVpssInit->pipeId, pstHbVinVpssInit->mipiIdx, pstHbVinVpssInit->vin_vps_mode);
	iRet = HB_SYS_SetVINVPSMode(pstHbVinVpssInit->pipeId, (SYS_VIN_VPS_MODE_E)pstHbVinVpssInit->vin_vps_mode);
	if(iRet < 0) 
	{
		printf("HB_SYS_SetVINVPSMode%d error!\n", pstHbVinVpssInit->vin_vps_mode);
		return iRet;
	}

	/* 创建VIN pipe */
	iRet = HB_VIN_CreatePipe(pstHbVinVpssInit->pipeId, &pstHbVinVpssInit->stPipeInfo);   // isp init
	if(iRet < 0)
	{
		printf("HB_VIN_CreatePipe error!\n");
		return iRet;
	}

	/* 设置mipi和dev绑定 */
	iRet = HB_VIN_SetMipiBindDev(pstHbVinVpssInit->pipeId, pstHbVinVpssInit->mipiIdx);
	if(iRet < 0) 
	{
		printf("HB_VIN_SetMipiBindDev error!\n");
		return iRet;
	}

	/* 设置dev 的 vc_index，使用 MIPI 的哪个 vc */
	iRet = HB_VIN_SetDevVCNumber(pstHbVinVpssInit->pipeId, pstHbVinVpssInit->deseri_port);
	if(iRet < 0) 
	{
		printf("HB_VIN_SetDevVCNumber error!\n");
		return iRet;
	}

	/* 设置dev的属性 */
	iRet = HB_VIN_SetDevAttr(pstHbVinVpssInit->pipeId, &pstHbVinVpssInit->stDevInfo);     // sif init
	if(iRet < 0) 
	{
		printf("HB_VIN_SetDevAttr error!\n");
		return iRet;
	}

	if (iNeedIsp)
	{

		/* 设置pipe属性 */
		iRet = HB_VIN_SetPipeAttr(pstHbVinVpssInit->pipeId, &pstHbVinVpssInit->stPipeInfo);     // isp init
		if(iRet < 0) 
		{
			printf("HB_VIN_SetPipeAttr error!\n");
			goto pipe_err;
		}
	
		#if 0
		iRet = HB_VIN_SetChnDISAttr(pstHbVinVpssInit->pipeId, 1, &disinfo);  //  dis init
		if(iRet < 0) {
			printf("HB_VIN_SetChnDISAttr error!\n");
			goto pipe_err;
		}

		iRet = HB_VIN_SetChnLDCAttr(pstHbVinVpssInit->pipeId, 1, &ldcinfo);   //  ldc init
		if(iRet < 0) {
			printf("HB_VIN_SetChnLDCAttr error!\n");
			goto pipe_err;
		}
		#endif

		iRet = HB_VIN_SetChnAttr(pstHbVinVpssInit->pipeId, 1);               //  dwe init
		if(iRet < 0) {
			printf("HB_VIN_SetChnAttr error!\n");
			goto chn_err;
		}
	
		HB_VIN_SetDevBindPipe(pstHbVinVpssInit->pipeId, pstHbVinVpssInit->pipeId);    //  bind init
	}

	return iRet;

pipe_err:
	HB_VIN_DestroyDev(pstHbVinVpssInit->pipeId);  // sif deinit
chn_err:
	HB_VIN_DestroyPipe(pstHbVinVpssInit->pipeId);  // isp && dwe deinit
	return iRet;
}


static void hb_vin_vps_deinit(int pipeId)
{
	HB_VIN_DestroyDev(pipeId);  // sif deinit && destroy
	HB_VIN_DestroyChn(pipeId, 1);  // dwe deinit
	HB_VIN_DestroyPipe(pipeId);  // isp deinit && destroy
}


static int hb_vin_vps_start(int pipeId, int iNeedIsp)
{
	int ret = 0;

	if (iNeedIsp)
	{
		ret = HB_VIN_EnableChn(pipeId, 0);  // dwe start
		if(ret < 0) {
			printf("HB_VIN_EnableChn error!\n");
			return ret;
		}

		ret = HB_VIN_StartPipe(pipeId);  // isp start
		if(ret < 0) {
			printf("HB_VIN_StartPipe error!\n");
			return ret;
		}
	}
	
	ret = HB_VIN_EnableDev(pipeId);  // sif start && start thread
	if(ret < 0) {
		printf("HB_VIN_EnableDev error!\n");
		return ret;
	}

	return ret;
}

static void hb_vin_vps_stop(int pipeId)
{
	HB_VIN_DisableDev(pipeId);    // thread stop && sif stop
	HB_VIN_StopPipe(pipeId);    // isp stop
	HB_VIN_DisableChn(pipeId, 1);   // dwe stop
}


/**
 * @brief			horizon_sensor_init
 * @note		
 * @param[in]		pstMipiSensorInfo
 * @param[in-out]	pstCamHandle
 * @return			0: success,	<0: failed
 */
static int hb_sensor_init(HB_SENSOR_INIT_S *pstHbSensorInit)
{
	int iRet = 0;

	if (!pstHbSensorInit)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	HB_MIPI_SetBus(&pstHbSensorInit->stMipiSensorInfo, pstHbSensorInit->bus_id);
	HB_MIPI_SetPort(&pstHbSensorInit->stMipiSensorInfo, pstHbSensorInit->port_id);
    HB_MIPI_SensorBindSerdes(&pstHbSensorInit->stMipiSensorInfo, pstHbSensorInit->sedres_index, pstHbSensorInit->sedres_port);
    HB_MIPI_SensorBindMipi(&pstHbSensorInit->stMipiSensorInfo,  pstHbSensorInit->mipiIdx);

	iRet = HB_MIPI_InitSensor(pstHbSensorInit->devId, &pstHbSensorInit->stMipiSensorInfo);
	if(iRet < 0) 
	{
		printf("HB_MIPI_InitSensor error!\n");
		return iRet;
	}
	printf("HB_MIPI_InitSensor end\n");

	return iRet;
}

static int hb_sensor_deinit(int devId)
{
	int ret = 0;

	ret = HB_MIPI_DeinitSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_DeinitSensor error!\n");
		return ret;
	}
	return ret;
}

static int hb_sensor_start(int devId)
{
	int ret = 0;

	ret = HB_MIPI_ResetSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_ResetSensor error!\n");
		return ret;
	}
	return ret;
}

static int hb_sensor_stop(int devId)
{
	int ret = 0;

	ret = HB_MIPI_UnresetSensor(devId);
	if(ret < 0) {
		printf("HB_MIPI_UnresetSensor error!\n");
		return ret;
	}
	return ret;
}


/**
 * @brief			horizon_mipi_init
 * @note		
 * @return			0: success,	<0: failed
 */
static int hb_mipi_init(MIPI_ATTR_S *pstMipiAttr, int mipiIdx)
{
	int iRet = 0;

	if (!pstMipiAttr)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	/* 设置mipi属性 */
	iRet = HB_MIPI_SetMipiAttr(mipiIdx, pstMipiAttr);
	if(iRet < 0) 
	{
		printf("HB_MIPI_SetDevAttr error!\n");
		return iRet;
	}
	
	printf("HB_MIPI_SetDevAttr end\n");
	return iRet;
}

static int hb_mipi_deinit(int mipiIdx)
{
	int ret = 0;

	printf("hb_sensor_deinit end==mipiIdx %d====\n", mipiIdx);
	ret = HB_MIPI_Clear(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_Clear error!\n");
		return ret;
	}
	printf("hb_mipi_deinit end======\n");
	return ret;
}

static int hb_mipi_start(int mipiIdx)
{
	int ret = 0;

	ret = HB_MIPI_ResetMipi(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_ResetMipi error!\n");
		return ret;
	}

	return ret;
}

static int hb_mipi_stop(int mipiIdx)
{
	int ret = 0;
	printf("hb_mipi_stop======\n");
	ret = HB_MIPI_UnresetMipi(mipiIdx);
	if(ret < 0) {
		printf("HB_MIPI_UnresetMipi error!\n");
		return ret;
	}
	return ret;
}


static void normal_buf_info_print(hb_vio_buffer_t * buf)
{
	printf("normal pipe_id (%d)type(%d)frame_id(%d)buf_index(%d)w x h(%dx%d) data_type %d img_format %d\n",
		buf->img_info.pipeline_id,
		buf->img_info.data_type,
		buf->img_info.frame_id,
		buf->img_info.buf_index,
		buf->img_addr.width,
		buf->img_addr.height,
		buf->img_info.data_type,
		buf->img_info.img_format);
}


int hb_vps_init(int pipeId, uint32_t vin_vps_mode)
{
	int ret = 0;
	VPS_GRP_ATTR_S grp_attr;
	VPS_CHN_ATTR_S chn_attr;
	VPS_PYM_CHN_ATTR_S pym_chn_attr;

	memset(&grp_attr, 0, sizeof(VPS_GRP_ATTR_S));
	grp_attr.maxW = RGB_WIDTH;
	grp_attr.maxH = RGB_HEIGHT;
	ret = HB_VPS_CreateGrp(pipeId, &grp_attr);
	if (ret) {
		printf("HB_VPS_CreateGrp error!!!\n");
	} else {
		printf("created a group ok:GrpId = %d\n", pipeId);
	}
	HB_SYS_SetVINVPSMode(pipeId, (SYS_VIN_VPS_MODE_E)vin_vps_mode);

	memset(&chn_attr, 0, sizeof(VPS_CHN_ATTR_S));
	chn_attr.enScale = 1;
	chn_attr.width = RGB_WIDTH;
	chn_attr.height = RGB_HEIGHT;
	chn_attr.frameDepth = 6;

	if (BIT2CHN(need_ipu, 1)) {
		ret = HB_VPS_SetChnAttr(pipeId, 1, &chn_attr);
	if (ret) {
		printf("HB_VPS_SetChnAttr error!!!\n");
	} else {
		printf("set chn Attr ok: GrpId = %d, chn_id = %d\n",
				 pipeId, 1);
	}
		HB_VPS_EnableChn(pipeId, 1);
	}

	struct HB_SYS_MOD_S src_mod, dst_mod;
	src_mod.enModId = HB_ID_VIN;
	src_mod.s32DevId = pipeId;  // pipe0
	src_mod.s32ChnId = 1;
	dst_mod.enModId = HB_ID_VPS;
	dst_mod.s32DevId = pipeId; // pipe1
	dst_mod.s32ChnId = 0;
	ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (ret != 0)
		printf("HB_SYS_Bind failed\n");

	return ret;
}

void hb_vps_deinit(int pipeId)
{
	HB_VPS_DestroyGrp(pipeId);
}

void hb_vps_start(int pipeId)
{
	int ret = 0;

	ret = HB_VPS_StartGrp(pipeId);
	if (ret) {
		printf("HB_VPS_StartGrp error!!!\n");
	} else {
		printf("start grp ok: grp_id = %d\n", pipeId);
	}
}

void hb_vps_stop(int pipeId)
{
	HB_VPS_StopGrp(pipeId);
}


/**
 * @brief			hb_venc_init
 * @note		
 * @return			0: success,	<0: failed
 */
static int hb_venc_common_init()
{
    int ret;

    ret = HB_VENC_Module_Init();
    if (ret) {
        printf("HB_VENC_Module_Init: %d\n", ret);
    }

    return ret;
}

static int hb_venc_common_deinit()
{
    int ret;

    ret = HB_VENC_Module_Uninit();
    if (ret) {
        printf("HB_VENC_Module_Uninit: %d\n", ret);
    }

    return ret;
}


static int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    int streambuf = 2*1024*1024;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;
    
    if (p_Width * p_Height > 2688 * 1522) {
        streambuf = 3 * 1024 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900*1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        streambuf = 2048 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        streambuf = 1536 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else if (p_Width * p_Height > 704 * 576) {
        streambuf = 512 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        streambuf = 256 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048*1024;
    }
    streambuf = (p_Width * p_Height)&0xfffff000;

		pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
		pVencChnAttr->stVencAttr.u32PicHeight = p_Height;

		pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
		pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
		pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

		pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;
		pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
		pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
		pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
		pVencChnAttr->stVencAttr.stAttrMjpeg.restart_interval = 0;
		pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;

		pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
		pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 6;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}


static int hb_venc_init(int VeChn, int type, int width, int height, int bitrate) 
{
	int ret;
	VENC_CHN_ATTR_S vencChnAttr;
	VENC_RC_ATTR_S *pstRcParam;

	VencChnAttrInit(&vencChnAttr, (PAYLOAD_TYPE_E)type, width, height, HB_PIXEL_FORMAT_NV12);

	ret = HB_VENC_CreateChn(VeChn, &vencChnAttr);
	if (ret) {
	      printf("HB_VENC_CreateChn %d failed, 0x%x\n", VeChn, ret);
	      return -1;
	}

	pstRcParam = &(vencChnAttr.stRcAttr);

	vencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGCBR;
	ret = HB_VENC_GetRcParam(VeChn, pstRcParam);
	if (ret) {
	      printf("HB_VENC_GetRcParam failed.\n");
	      return -1;
	}
	printf("vencChnAttr.stRcAttr.enRcMode=%d\n", vencChnAttr.stRcAttr.enRcMode);
	pstRcParam->stMjpegCbr.u32FrameRate = 30;
	pstRcParam->stMjpegCbr.u32MinQP = minQP;
	pstRcParam->stMjpegCbr.u32MaxQP = maxQP;
	pstRcParam->stMjpegCbr.u32BitRate = bitrate;

	ret = HB_VENC_SetRcParam(VeChn, &vencChnAttr.stRcAttr);
	if (ret) {
	      printf("HB_VENC_SetRcParam failed.\n");
	      return -1;
	}

	//hb_mm_mc_configure
	ret = HB_VENC_SetChnAttr(VeChn, &vencChnAttr);
	if (ret) {
	      printf("HB_VENC_SetChnAttr failed.\n");
	      return -1;
	}

	return 0;
}

static int hb_venc_deinit(int VeChn)
{
    int s32Ret = 0;

    s32Ret = HB_VENC_DestroyChn(VeChn);
    if (s32Ret) {
        printf("HB_VENC_DestroyChn failed\n");
        return -1;
    }

    return 0;
}

int hb_venc_start(int VeChn) {
        int ret = 0;
        VENC_RECV_PIC_PARAM_S recvParam;
        memset(&recvParam, 0x00, sizeof(VENC_RECV_PIC_PARAM_S));
        recvParam.s32RecvPicNum = 0;

        ret = HB_VENC_StartRecvFrame(VeChn, &recvParam);
        if (ret) {
                printf("HB_VENC_StartRecvFrame failed.\n");
                return -1;
        }

        return 0;
}

int hb_venc_stop(int VeChn) {
        int ret = 0;
        ret = HB_VENC_StopRecvFrame(VeChn);
        if (ret) {
                printf("HB_VENC_StopRecvFrame failed\n");
                return -1;
        }

        return 0;
}

int hb_vps_bind_venc(int vpsGrp, int vpsChn, int vencChn)
{
    int s32Ret = 0;
    struct HB_SYS_MOD_S src_mod, dst_mod;

	src_mod.enModId = HB_ID_VPS;
	src_mod.s32DevId = vpsGrp;
	src_mod.s32ChnId = vpsChn;
	dst_mod.enModId = HB_ID_VENC;
	dst_mod.s32DevId = vencChn;
	dst_mod.s32ChnId = 0;
	s32Ret = HB_SYS_Bind(&src_mod, &dst_mod);
	if (s32Ret != 0)
		printf("HB_SYS_Bind failed\n");

    return s32Ret;
}


/**
 * @brief			hb_video_init
 * @note		
 * @return			0: success,	<0: failed
 */
int hb_video_init(HB_VIDEO_DEV_S *pstHbVideoDev)
{
	int iRet = 0;
	uint64_t mmz_paddr;
	char* mmz_vaddr;
	int32_t mmz_size = RGB_WIDTH*RGB_HEIGHT*3/2;
	
	HB_VIN_VPSS_INIT_S *pstHbVinVpssInit;
	HB_SENSOR_INIT_S *pstHbSensorInit;
	MIPI_ATTR_S *pstMipiAttr;

	pstHbVinVpssInit = &pstHbVideoDev->stHbVinVpssInit;
	pstHbSensorInit = &pstHbVideoDev->stHbSensorInit;
	pstMipiAttr = &pstHbVideoDev->stMipiAttr;

	if (!pstHbVideoDev)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (pstHbVideoDev->isInit)
	{
		printf("[%s] Have already init\n", __func__);
		return 0;
	}

	hb_enable_sensor_clk(pstHbVideoDev->mipiIdx);

	iRet = hb_vin_vps_init(pstHbVinVpssInit, pstHbVideoDev->iNeedIsp);
	if (iRet)
	{
		printf("[%s] hb_vin_vps_init failed\n", __func__);
		return -1;
	}

	iRet = hb_sensor_init(pstHbSensorInit);
	if (iRet)
	{
		printf("[%s] horizon_sensor_init failed\n", __func__);
		goto err_vin_vpss_deinit;
	}

	iRet = hb_mipi_init(pstMipiAttr, pstHbVideoDev->mipiIdx);
	if (iRet)
	{
		printf("[%s] hb_mipi_init failed\n", __func__);
		goto err_sensor_deinit;
	}

	pstHbVideoDev->vin_fd = HB_VIN_GetChnFd(pstHbVideoDev->pipeId, 0);
	if(pstHbVideoDev->vin_fd < 0) 
	{
		printf("HB_VIN_GetChnFd error!!!\n");
	}
	else
	{
		printf("pstHbVideoDev->vin_fd = %d\n", pstHbVideoDev->vin_fd);
	}

	if (pstHbVideoDev->iNeedInitVps)
	{
		iRet = hb_vps_init(pstHbVideoDev->pipeId, pstHbVinVpssInit->vin_vps_mode);
		if(iRet < 0) {
			printf("hb_vps_init error!\n");
			goto err_mipi_deinit;
		}
	}

	if (pstHbVideoDev->iNeedInitVenc)
	{
		hv_vp_init();
	
		iRet = hb_venc_common_init();
		if (iRet) {
			printf("hb_venc_common_init failed\n");
			goto err_mipi_deinit;
		}

		iRet = hb_venc_init(pstHbVideoDev->VeChn, PT_MJPEG, RGB_WIDTH, RGB_HEIGHT, 3000);
		if (iRet) {
			printf("hb_venc_init failed\n");
			goto err_venc_common_deinit;
		}
	
		iRet = HB_SYS_Alloc(&mmz_paddr, (void **)&mmz_vaddr, mmz_size);
		if (iRet == 0) {
			printf("mmzAlloc paddr = 0x%x, vaddr = 0x%x\n", mmz_paddr, mmz_vaddr);

			pstHbVideoDev->stMemBlock.isAvailable = 1;
			pstHbVideoDev->stMemBlock.mmz_paddr = mmz_paddr;
			pstHbVideoDev->stMemBlock.mmz_vaddr = mmz_vaddr;
		}
		else
		{
			printf("HB_SYS_Alloc failed, iRet = %d\n", iRet);
			goto err_venc_deinit;
		}
	}

	pstHbVideoDev->isInit = 1;
	return 0;

err_venc_deinit:
	if (pstHbVideoDev->iNeedInitVenc)
	{
		hb_venc_deinit(pstHbVideoDev->VeChn);
	}
err_venc_common_deinit:
	if (pstHbVideoDev->iNeedInitVenc)
	{
		hb_venc_common_deinit();
	}
err_mipi_deinit:
	hb_mipi_deinit(pstHbVideoDev->mipiIdx);
err_sensor_deinit:
	hb_sensor_deinit(pstHbSensorInit->devId);
err_vin_vpss_deinit:
	hb_vin_vps_deinit(pstHbVinVpssInit->pipeId);
		
	return iRet;
}

int hb_video_Uninit(HB_VIDEO_DEV_S *pstHbVideoDev)
{
	int iRet = 0;

	if (!pstHbVideoDev)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (0 == pstHbVideoDev->isInit)
	{
		return 0;
	}

	if (pstHbVideoDev->iNeedInitVenc)
	{
		hb_venc_deinit(pstHbVideoDev->VeChn);	
		hb_venc_common_deinit();

		iRet = HB_SYS_Free(pstHbVideoDev->stMemBlock.mmz_paddr, pstHbVideoDev->stMemBlock.mmz_vaddr);
		if (iRet == 0) {
			printf("mmzFree paddr = 0x%x, vaddr = 0x%x\n", pstHbVideoDev->stMemBlock.mmz_paddr,
			pstHbVideoDev->stMemBlock.mmz_vaddr);
		}
		
		hv_vp_deinit();
	}
	
	if (pstHbVideoDev->iNeedInitVps)
	{
		hb_vps_deinit(pstHbVideoDev->pipeId);	
	}

	hb_sensor_deinit(pstHbVideoDev->devId);
	hb_mipi_deinit(pstHbVideoDev->mipiIdx);
	hb_vin_vps_deinit(pstHbVideoDev->pipeId);

	pstHbVideoDev->isInit = 0;
	return 0;
}


/**
 * @brief			hb_video_start_stream
 * @note		
 * @return			0: success,	<0: failed
 */
int hb_video_start_stream(HB_VIDEO_DEV_S *pstHbVideoDev)
{
	int ret = 0;

	if (!pstHbVideoDev)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	hb_vin_vps_start(pstHbVideoDev->pipeId, pstHbVideoDev->iNeedIsp);

	if (pstHbVideoDev->iNeedInitVps)
	{
		hb_vps_start(pstHbVideoDev->pipeId);	
	}

	if (pstHbVideoDev->iNeedInitVenc)
	{
		hb_venc_start(pstHbVideoDev->VeChn);		
	}

	hb_sensor_start(pstHbVideoDev->devId);
	hb_mipi_start(pstHbVideoDev->mipiIdx);

	pstHbVideoDev->is_streaming = 1;
	return 0;
}


int hb_video_stop_stream(HB_VIDEO_DEV_S *pstHbVideoDev)
{
	if (!pstHbVideoDev)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (0 == pstHbVideoDev->is_streaming)
		return 0;

	pstHbVideoDev->is_streaming = 0;
	usleep(100*1000);

	if (pstHbVideoDev->iNeedInitVenc)
	{
		hb_venc_stop(pstHbVideoDev->VeChn);		
	}

	if (pstHbVideoDev->iNeedInitVps)
	{
		hb_vps_stop(pstHbVideoDev->pipeId);	
	}

	hb_sensor_stop(pstHbVideoDev->devId);
	hb_mipi_stop(pstHbVideoDev->mipiIdx);
	hb_vin_vps_stop(pstHbVideoDev->pipeId);

	return 0;
}


/**
 * @brief			hb_get_raw_data
 * @note		
 * @return			0: success,	<0: failed
 */
int hb_get_raw_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo)
{
	int iRet = 0;
	hb_vio_buffer_t sif_raw;
	fd_set readfd;	
	struct timeval select_timeout = {0};

	if (!pstHbVideoDev || !pstImageDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (!pstHbVideoDev->is_streaming)
	{
		return 1;
	}

	FD_ZERO(&readfd);
	FD_SET(pstHbVideoDev->vin_fd, &readfd);
	select_timeout.tv_usec = 500 * 1000;
	memset(&sif_raw, 0, sizeof(sif_raw));

#if 0
	iRet = select(pstHbVideoDev->vin_fd + 1, &readfd, NULL, NULL, &select_timeout);
	if (iRet == -1) 
	{
		printf("select_isp_dump_yuv_func select error\n");
	} 
	else if (iRet)
	{
#endif
		iRet = HB_VIN_GetDevFrame(pstHbVideoDev->pipeId, 0, &sif_raw, 2000);
		if (iRet < 0) 
		{
			printf("HB_VIN_GetDevFrame error!!!\n");
			return -1;
		}
		else
		{
			//normal_buf_info_print(&sif_raw);
			HB_VIN_ReleaseDevFrame(pstHbVideoDev->pipeId, 0, &sif_raw);
		}

		pstImageDataInfo->uiImageSize= sif_raw.img_addr.stride_size * sif_raw.img_addr.height;
		pstImageDataInfo->pucImageData= (unsigned char*)sif_raw.img_addr.addr[0];
		pstImageDataInfo->timeStamp = sif_raw.img_info.tv.tv_sec*1000000 + sif_raw.img_info.tv.tv_usec;
		pstImageDataInfo->uiFrameCnt = sif_raw.img_info.frame_id;
#if 0

	}
	else
	{
		printf("[%s] select time out\n", __func__);
		return -1;
	}
#endif
	
	return iRet;
}

int hb_get_yuv_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo)
{
	int iRet;
	unsigned int size_y, size_uv;
	hb_vio_buffer_t isp_yuv;
	fd_set readfd;	
	struct timeval select_timeout = {0};

	if (!pstHbVideoDev || !pstImageDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (!pstHbVideoDev->is_streaming)
	{
		return 1;
	}

	FD_ZERO(&readfd);
	FD_SET(pstHbVideoDev->vin_fd, &readfd);
	select_timeout.tv_usec = 500 * 1000;
	memset(&isp_yuv, 0, sizeof(isp_yuv));

#if 1
	iRet = select(pstHbVideoDev->vin_fd + 1, &readfd, NULL, NULL, &select_timeout);
	if (iRet == -1) 
	{
		printf("select_isp_dump_yuv_func select error\n");
	} 
	else if (iRet)
	{
#endif
		iRet = HB_VIN_GetChnFrame(pstHbVideoDev->pipeId, 0, &isp_yuv, 2000);
		if (iRet < 0) 
		{
			printf("HB_VIN_GetChnFrame error!!!\n");
		} 
		else 
		{
			iRet = HB_VIN_ReleaseChnFrame(pstHbVideoDev->pipeId, 0, &isp_yuv);
			if (iRet < 0) 
			{
				printf("HB_VPS_ReleaseChnFrame error!!!\n");
			}
		}

		//normal_buf_info_print(&isp_yuv);

		size_y = isp_yuv.img_addr.width * isp_yuv.img_addr.height;
		size_uv = size_y / 2;
		pstImageDataInfo->uiImageSize= size_y + size_uv;
		
		memset(pu8YuvBuffer, 0, sizeof(pu8YuvBuffer));
		memcpy(pu8YuvBuffer, isp_yuv.img_addr.addr[0], size_y);
		memcpy(pu8YuvBuffer+size_y, isp_yuv.img_addr.addr[1], size_uv);
		pstImageDataInfo->pucImageData= pu8YuvBuffer;
		
		pstImageDataInfo->timeStamp = isp_yuv.img_info.tv.tv_sec*1000000 + isp_yuv.img_info.tv.tv_usec;
		pstImageDataInfo->uiFrameCnt = isp_yuv.img_info.frame_id;
		
#if 1
	}
	else
	{
		printf("[%s] select time out\n", __func__);
		return -1;
	}
#endif

	return iRet;
}


int hb_encode_send_frame(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo)
{
	int ret = 0;
	int32_t offset;
	VIDEO_FRAME_S stFrame;

	if (!pstHbVideoDev || !pstImageDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	memset(&stFrame, 0x00, sizeof(VIDEO_FRAME_S));
	stFrame.stVFrame.width = RGB_WIDTH;
	stFrame.stVFrame.height = RGB_HEIGHT;
	stFrame.stVFrame.size = RGB_WIDTH*RGB_HEIGHT*3/2;
	stFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
	offset = RGB_WIDTH * RGB_HEIGHT;

	if (1 == pstHbVideoDev->stMemBlock.isAvailable)
	{
		stFrame.stVFrame.phy_ptr[0] = pstHbVideoDev->stMemBlock.mmz_paddr;
		stFrame.stVFrame.phy_ptr[1] = pstHbVideoDev->stMemBlock.mmz_paddr + offset;
		stFrame.stVFrame.phy_ptr[2] = pstHbVideoDev->stMemBlock.mmz_paddr + offset * 5 / 4;
		stFrame.stVFrame.vir_ptr[0] = pstHbVideoDev->stMemBlock.mmz_vaddr;
		stFrame.stVFrame.vir_ptr[1] = pstHbVideoDev->stMemBlock.mmz_vaddr + offset;
		stFrame.stVFrame.vir_ptr[2] = pstHbVideoDev->stMemBlock.mmz_vaddr + offset * 5 / 4;

		memcpy(pstHbVideoDev->stMemBlock.mmz_vaddr, pstImageDataInfo->pucImageData, pstImageDataInfo->uiImageSize);

		ret = HB_VENC_SendFrame(pstHbVideoDev->VeChn, &stFrame, 3000);
	    if (ret < 0) {
	    	printf("HB_VENC_SendFrame failed.\n");
			return -1;
		}
	}
	else
	{
		printf("[%s] Hb mem is not available\n", __func__);
		return -1;
	}

	return 0;
}


int hb_get_encode_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo)
{
	int ret = 0;
	VIDEO_STREAM_S stStream;

	if (!pstHbVideoDev || !pstImageDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if (!pstHbVideoDev->is_streaming)
	{
		return 1;
	}

	memset(&stStream, 0x00, sizeof(VIDEO_STREAM_S));
	ret = HB_VENC_GetStream(pstHbVideoDev->VeChn, &stStream, 1000);
    if (ret < 0) 
	{
    	printf("HB_VENC_GetStream failed. ret = 0x%x\n", ret);
		return -1;
    } 
	else 
    {
		HB_VENC_ReleaseStream(pstHbVideoDev->VeChn, &stStream);

		pstImageDataInfo->pucImageData= (unsigned char*)stStream.pstPack.vir_ptr;
		pstImageDataInfo->uiImageSize= stStream.pstPack.size;
		pstImageDataInfo->timeStamp = stStream.pstPack.pts;
    }

	return 0;
}


int hb_set_fps(HB_VIDEO_DEV_S *pstHbVideoDev, unsigned int u32Fps)
{
	int iRet = 0;

	iRet = HB_MIPI_SwSensorFps(pstHbVideoDev->devId, u32Fps);
	if (iRet)
	{
		printf("[%s] HB_MIPI_SwSensorFps[%u] failed, iRet = 0x%x\n", __func__, u32Fps, iRet);
		return iRet;
	}

	return iRet;
}


