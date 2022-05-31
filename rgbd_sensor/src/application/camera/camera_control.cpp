#include <semaphore.h>
#include <arm_neon.h>

#include "camera_control.h"
#include "tof_depth_process.h"
// #include "v4l2_capture.h"
#include "camera_hb_cfg.h"


#define TOF_CALIB_OFFSET		(0x2100)
#define RGBD_CALIB_OFFSET		(0x1000)


static int R1[256];
static int G1[256];
static int G2[256];
static int B1[256];

static unsigned char au8RawBuffer[RAW_SIZE];
static char str_calib_name[128] = {0};////�궨�ļ�·��
static char str_removeINS_name[128] = {0};////�궨�ļ�·��
static unsigned char p_tof_eeprom_double[128*1024+RINS_FILE_SIZE] = {0};


unsigned long long get_tick_count()
{
	unsigned long long tick = 0;

	struct timeval tv;
	gettimeofday(&tv, 0);
	tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);

	return tick;
}


static void test_tof_fps(void)
{
    static int frames_num = 0;
    static long start_time = 1;

    if (start_time == 1) {
        start_time = get_tick_count();
    }

    frames_num++;

    if (frames_num % 10 == 0) {
        float cur_framerate = 0;
        cur_framerate = (float)frames_num * 1000 / (get_tick_count() - start_time);

        printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        printf("TOf Frame Rate = %.2f\n", cur_framerate);
        start_time = get_tick_count();
        frames_num = 0;
    }
}


int irs2381c_start(I2C_CB_S *pstI2cCb)
{
	unsigned short start_value = 0x0001;
	
	I2cWrite(pstI2cCb, 0x9400, start_value, I2C_FMT_A16D16);
	return 0;
}

int irs2381c_stop(I2C_CB_S *pstI2cCb)
{
	unsigned short start_value = 0;
	
	I2cWrite(pstI2cCb, 0x9400, start_value, I2C_FMT_A16D16);
	return 0;
}

static int Read_FLASH_Hal(I2C_CB_S *pstI2cCb, unsigned char *pcBuf, int start_addr, int len)
{
	int spiReadAddr = 0x0100;
	unsigned short spiWriteAddr = 0x0000;
	int SF_ReadCMD = 0x03;
	int SF_ReadLen = len;//128 * 1024;
	int SF_StartAddr = start_addr;
	unsigned short u16Status;
	int value;
	int i;
	int offset = 0;
	int pageSize = 256;
	
	int readLen = (SF_ReadLen - offset) > pageSize ? pageSize : (SF_ReadLen - offset);
	int cmdReadLen = readLen + 3;
	int readAddr = SF_StartAddr + offset;

	unsigned char *data = pcBuf;
	if (!pcBuf)
	{
		printf("NULL ptr!\n");
		return -1;
	}

	irs2381c_stop(pstI2cCb);
	usleep(500000);

	while (readLen > 0)
	{
		#if 1
		/* read status info */
		I2cRead(pstI2cCb, 0x9403, &u16Status, I2C_FMT_A16D16);

		/* read ICM status info */
		I2cRead(pstI2cCb, 0x8423, &u16Status, I2C_FMT_A16D16);

		/* read status info */
		I2cRead(pstI2cCb, 0x9403, &u16Status, I2C_FMT_A16D16);
		I2cRead(pstI2cCb, 0xA08C, &u16Status, I2C_FMT_A16D16);
		#endif

		u16Status = ((SF_ReadCMD << 8) | ((readAddr >> 16) & 0xFF)) & 0xFFFF;
		I2cWrite(pstI2cCb, spiWriteAddr, u16Status, I2C_FMT_A16D16);
		
		u16Status = readAddr & 0xFFFF;
		I2cWrite(pstI2cCb, spiWriteAddr + 1, u16Status, I2C_FMT_A16D16);
		
		u16Status = 0xC001;
		I2cWrite(pstI2cCb, 0xA087, u16Status, I2C_FMT_A16D16);
		
		u16Status = spiWriteAddr;
		I2cWrite(pstI2cCb, 0xA088, u16Status, I2C_FMT_A16D16);
		u16Status = spiReadAddr;
		I2cWrite(pstI2cCb, 0xA089, u16Status, I2C_FMT_A16D16);
		u16Status = (0x7 << 13) | (cmdReadLen & 0x3FF);
		I2cWrite(pstI2cCb, 0xA08A, u16Status, I2C_FMT_A16D16);

		u16Status = 0x0002;
		I2cWrite(pstI2cCb, 0xA08B, u16Status, I2C_FMT_A16D16);

		do
		{
			usleep(500);	/* ���뱣�� */
				
		} while (u16Status & 0x1);

	#if 0
		/* read spi data from read buffer */
		for (i = 0; i < readLen / 2; i++)
		{
			I2cRead(pstI2cCb, spiReadAddr + i, &value, I2C_FMT_A16D16);
			data[offset + i * 2] = (value >> 8) & 0xFF;
			data[offset + i * 2 + 1] = value & 0xFF;
		}
		/* last byte is spi len is odd */
		if (i * 2 < readLen)
		{
			I2cRead(pstI2cCb, spiReadAddr + i, &value, I2C_FMT_A16D16);
			data[offset + i * 2] = (value >> 8) & 0xFF;
		}
	#else
		i2c_block_read_eeprom_a16_d8(pstI2cCb, spiReadAddr, data, pageSize);
		data = data + pageSize;	
	#endif
	
		offset += readLen;
		readLen = (SF_ReadLen - offset) > pageSize ? pageSize : (SF_ReadLen - offset);
		cmdReadLen = readLen + 3;
		readAddr = SF_StartAddr + offset;
	}

	irs2381c_start(pstI2cCb);
	
	return 0;
}


int __ReadCalibData(I2C_CB_S *pstI2cCb, char *pcCalibDataPath, int iStartAddr, int iLen)
{
	unsigned char *pTmpBuf = p_tof_eeprom_double;
	FILE *cali_fb = NULL;
	FILE *RINS_fb = NULL;
	int i, ret=-1, retry = 0;
	
	memset(pTmpBuf, 0xFF, MAX_READ_SZIE);
	
	printf("read cailb begin\n");
	Read_FLASH_Hal(pstI2cCb, pTmpBuf, iStartAddr, iLen);

	cali_fb = fopen(pcCalibDataPath, "wb");
	if(cali_fb == NULL)
	{
		printf("fopen %s error...... !\n", pcCalibDataPath);
		return -1;
	}
	
	fwrite(pTmpBuf, iLen, 1, cali_fb);///MAX_READ_SZIE    MAX_READ_PAGE
	fclose(cali_fb);
	sync();
	printf("read cailb end\n");	
	return 0;
}



static int ReadSensorID(I2C_CB_S *pstI2cCb, unsigned short au16SensorId[4])
{
	I2cRead(pstI2cCb, 0xA097, &au16SensorId[0], I2C_FMT_A16D16);
	I2cRead(pstI2cCb, 0xA098, &au16SensorId[1], I2C_FMT_A16D16);
	I2cRead(pstI2cCb, 0xA099, &au16SensorId[2], I2C_FMT_A16D16);
	I2cRead(pstI2cCb, 0xA09A, &au16SensorId[3], I2C_FMT_A16D16);

	printf("sensorID: %x-%x-%x-%x \n\r", au16SensorId[0], au16SensorId[1], au16SensorId[2], au16SensorId[3]);
	return 0;
}


static int ReadCalibData(I2C_CB_S *pstI2cCb, char *pcSavePath, int iStartAddr, int iLen)
{
	int iRet = 0;
	FILE *pFile;

	if (!pstI2cCb || !pcSavePath)
	{
		printf("[%s] NULL ptr!", __func__);
		return -1;
	}

	pFile = fopen(pcSavePath, "rb");
	if(!pFile)
	{
		printf("there is no local calib_file ,read from flash\n\n");
		iRet = __ReadCalibData(pstI2cCb, pcSavePath, iStartAddr, iLen);
	}
	else
	{
		printf("find local calib_file\n\n");
	}

	return iRet;
}


int RawConvert129to173(unsigned char *input_raw, unsigned char *output_raw)
{
	int i = 0;

	for (i = 0; i < 17; i++)
	{
		memcpy(output_raw+i*224*173*2, input_raw+i*224*129*2, 224*2);
		memcpy(output_raw+i*224*173*2+224*23*2, input_raw+i*224*129*2+224*2, 224*128*2);
	}

	return 0;
}


int data_convert(unsigned char* data,unsigned char* buffer_out,unsigned int num)
{
  //unsigned short* buffer_in = buffer_out;
  //unsigned short* buffer_nn = buffer_in;
  unsigned char* data_in = data;

  uint8x8x2_t p;
  uint8x8_t P_One,p_Two;
  uint8_t p1[8];
  uint8_t p2[8];

  unsigned int num_in = num ;
 
  unsigned int k = 0;

  uint16x8_t value_hi,value_lo;
  uint16x8_t value_hi_tmp,value_lo_tmp;
  uint16x8_t value_lo_tmp_1,value_lo_tmp_w;
  
  uint16x8_t a,b,c,d,e;
  
  uint16x8_t buffer_out_tmp_a,buffer_out_tmp_b,
  buffer_out_tmp_c,buffer_out_tmp_d;
  
  uint16x8_t buffer_out_tmp_1,buffer_out_tmp_2,
  buffer_out_tmp_3;
  
  uint16x8_t buffer_out_tmp_all;
  
  a = vdupq_n_u16(0xff00);
  b = vdupq_n_u16(0xff);
  c = vdupq_n_u16(0x000f);
  d = vdupq_n_u16(0xf000);
  e = vdupq_n_u16(0x0fff);
  for(k = 0;k < num_in;k += 16)
  {
	p = vld2_u8(data_in + k);
	
	vst1_u8(p1,p.val[0]);
	vst1_u8(p2,p.val[1]);

	P_One = vld1_u8(p1);
	p_Two = vld1_u8(p2);

	value_hi_tmp = vshll_n_u8(P_One,8);
	value_hi = vandq_u16(value_hi_tmp,a);
	
	//value_lo_tmp_1 = vshll_n_u8(p_Two,0);
	value_lo_tmp_w = vshll_n_u8(p_Two,1);
	value_lo_tmp_1 = vshrq_n_u16(value_lo_tmp_w,1);
	
	value_lo_tmp = vandq_u16(value_lo_tmp_1,b);

	buffer_out_tmp_a = vorrq_u16(value_hi_tmp,value_lo_tmp);

	buffer_out_tmp_b = vandq_u16(buffer_out_tmp_a,c);

	buffer_out_tmp_c = vshlq_n_u16(buffer_out_tmp_b,12);
	
	buffer_out_tmp_d = vandq_u16(buffer_out_tmp_c,d);

	buffer_out_tmp_1 = vorrq_u16(value_hi_tmp,value_lo_tmp);
	buffer_out_tmp_2 = vshrq_n_u16(buffer_out_tmp_1,4);
	buffer_out_tmp_3 = vandq_u16(buffer_out_tmp_2,e);
	
	buffer_out_tmp_all = vorrq_u16(buffer_out_tmp_d,buffer_out_tmp_3);
	vst1q_u16((unsigned short*)(buffer_out + k),buffer_out_tmp_all);

//		data_in += 16;
//	  buffer_in += 8;
	
  }
  
  //memcpy(data,(void*)buffer_nn,num);

  return 0;
}


int GetImageData(void *pHandle, IMAGE_DATA_INFO_S *pstImageDataInfo)
{
	int iRet = 0;
	unsigned int uiFrameCnt = 0;
	camera_handle *pstCameraHandler = (camera_handle*)pHandle;

	if (!pHandle || !pstImageDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	if ((CAM_TYPE_TOF == pstCameraHandler->eCamType) || (CAM_TYPE_TOF_RGBD == pstCameraHandler->eCamType))
		pstImageDataInfo->ePixelFormat = PIXEL_FORMAT_RAW;
	else if (CAM_TYPE_RGB == pstCameraHandler->eCamType)
		pstImageDataInfo->ePixelFormat = PIXEL_FORMAT_YUV;	

	switch (pstImageDataInfo->ePixelFormat)
	{
		case PIXEL_FORMAT_RAW:
		#ifdef HORIZON_PLATFORM
			iRet = hb_get_raw_data(&pstCameraHandler->stHbVideoDev, pstImageDataInfo);
		#else
			iRet = v4l2_get_raw_data(pstCameraHandler->pstV4l2Dev, pstImageDataInfo);
		#endif
			break;

		case PIXEL_FORMAT_YUV:
		#ifdef HORIZON_PLATFORM
			iRet = hb_get_yuv_data(&pstCameraHandler->stHbVideoDev, pstImageDataInfo);
		#endif
			break;
		
		case PIXEL_FORMAT_MJPEG:
		case PIXEL_FORMAT_H264:
		#ifdef HORIZON_PLATFORM
			iRet = hb_get_encode_data(&pstCameraHandler->stHbVideoDev, pstImageDataInfo);
		#endif
			break;

		default:
			printf("Unkown pixel format: 0x%x\n", pstImageDataInfo->ePixelFormat);
			return -1;
	}

	if (iRet < 0)
	{
		printf("[%s] get image data failed, pixel format = 0x%x\n", __func__, pstImageDataInfo->ePixelFormat);
		return -1;
	}
	else if (iRet > 0)
	{
		return -1;
	}

	if (PIXEL_FORMAT_RAW == pstImageDataInfo->ePixelFormat)
	{
	#ifdef HORIZON_PLATFORM
		int byte_index,pixel_index,buffer_size = 0;
		unsigned char *srcBuf = pstImageDataInfo->pucImageData;
		unsigned short *buffer = (unsigned short *)au8RawBuffer;
		
		for(byte_index = 0, pixel_index = 0; byte_index < (pstImageDataInfo->uiImageSize-3); )
		{
			buffer[pixel_index+0] = (((short)(srcBuf[byte_index+0]))<<4) & 0x0FF0;
			buffer[pixel_index+0] = buffer[pixel_index+0] | (short)((srcBuf[byte_index+2]>>0) & 0x000F);
			buffer[pixel_index+1] = (((short)(srcBuf[byte_index+1]))<<4) & 0x0FF0;
			buffer[pixel_index+1] = buffer[pixel_index+1] | (short)((srcBuf[byte_index+2]>>4) & 0x000F);
		
			byte_index = byte_index + 3;
			pixel_index = pixel_index + 2;
		}

		pstImageDataInfo->pucImageData = au8RawBuffer;
		pstImageDataInfo->uiImageSize = pstImageDataInfo->uiImageSize*4/3;
	#else
		if ((CAM_TYPE_TOF == pstCameraHandler->eCamType) || (CAM_TYPE_TOF_RGBD == pstCameraHandler->eCamType))
		{
			if (PIPELINE_ISP == pstCameraHandler->ePipelineType)
			{
				data_convert(pstImageDataInfo->pucImageData, pstImageDataInfo->pucImageData, pstImageDataInfo->uiRawSize);
			}
		}
	#endif
	}
#if 0
	printf("[%s] get 0x%x-%s image, w:h=%d:%d size = %u\n", __func__, 
		pstCameraHandler->eCamType,
		pstCameraHandler->acDirection,
		pstImageDataInfo->width,
		pstImageDataInfo->height,
		pstImageDataInfo->uiImageSize);
#endif
	return 0;
}


int FrameEncode(void *pHandle, IMAGE_DATA_INFO_S *pstImageInput, IMAGE_DATA_INFO_S *pstImageOutput)
{
	int iRet = 0;
	camera_handle *pstCameraHandler;

	if (!pHandle || !pstImageInput || !pstImageOutput)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstCameraHandler = (camera_handle*)pHandle;

	#ifdef HORIZON_PLATFORM
		iRet = hb_encode_send_frame(&pstCameraHandler->stHbVideoDev, pstImageInput);
		if (iRet)
		{
			printf("hb_encode_send_frame failed\n");
			return -1;
		}
	
		//usleep(300000);
		
		iRet = hb_get_encode_data(&pstCameraHandler->stHbVideoDev, pstImageOutput);
		if (iRet)
		{
			printf("hb_get_encode_data failed\n");
			return -1;
		}
	#else
	#endif
	
	return 0;
}


int GetSensorID(void *pCamHandle, unsigned short au16SensorID[4])
{
	camera_handle *pstCameraHandler;

	if (!pCamHandle)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstCameraHandler = (camera_handle*)pCamHandle;
	memcpy(au16SensorID, pstCameraHandler->au16SensorId, sizeof(pstCameraHandler->au16SensorId));

	return 0;
}


unsigned int GetSupportTofFilter(void *pCamHandle)
{
	camera_handle *pstCameraHandler;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;

	if (!pCamHandle)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstCameraHandler = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstCameraHandler->stDepthHandleCb[0];

	return pstDepthHandleCb->uiSupportedTofFilter;
}


int GetTofFilterStatus(void *pCamHandle, unsigned int *puiEnableTypeList, unsigned int *puiDisableTypeList)
{
	unsigned int i, type;
	SBOOL bEnable;
	camera_handle *pstCameraHandler;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;

	if (!pCamHandle || !puiEnableTypeList || !puiDisableTypeList)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstCameraHandler = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstCameraHandler->stDepthHandleCb[0];
	*puiEnableTypeList = 0;
	*puiDisableTypeList = 0;

	for (i = 0; i < 32; i++)
	{
		type = (1 << i);
		if (0 != (pstDepthHandleCb->uiSupportedTofFilter & type))
		{
			TOFM_GetTofFilter(pstDepthHandleCb->hTofMod, (TOF_FILTER)type, &bEnable);

			if (bEnable == true)
			{
				*puiEnableTypeList |= type;
			}
			else
			{
				*puiDisableTypeList |= type;
			}
		}
	}

	return 0;
}


int SetTofFilterStatus(void *pCamHandle, unsigned int *puiEnableTypeList, unsigned int *puiDisableTypeList)
{
	unsigned int i, type;
	camera_handle *pstCameraHandler;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb_1;

	if (!pCamHandle)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstCameraHandler = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstCameraHandler->stDepthHandleCb[0];
	pstDepthHandleCb_1 = &pstCameraHandler->stDepthHandleCb[1];

	for (i = 0; i < 32; i++)
	{
		type = (1 << i);
		if (0 != (pstDepthHandleCb->uiSupportedTofFilter & type))
		{	
			if (*puiEnableTypeList & type)
			{
				TOFM_SetTofFilter(pstDepthHandleCb->hTofMod, (TOF_FILTER)type, true);
				TOFM_SetTofFilter(pstDepthHandleCb_1->hTofMod, (TOF_FILTER)type, true);
			}
			else if (*puiDisableTypeList & type)
			{
				TOFM_SetTofFilter(pstDepthHandleCb->hTofMod, (TOF_FILTER)type, false);
				TOFM_SetTofFilter(pstDepthHandleCb_1->hTofMod, (TOF_FILTER)type, false);
			}
		}
	}

	return 0;
}

int StartCamera(void *pCamHdl)
{
	camera_handle *pstCameraHandler = (camera_handle*)pCamHdl;
	if (pstCameraHandler) {
		if (ENST_CAMERA_START == pstCameraHandler->nWorkState)
			return 0;
		int nRet = hb_video_start_stream(&pstCameraHandler->stHbVideoDev);
		printf("\n[wuwl-%s]->camT=%d, ret=%d.\n", __func__, pstCameraHandler->eCamType, nRet);
		pstCameraHandler->nWorkState = ENST_CAMERA_START;
		return nRet;
	}
	return -1;
}
int StopCamera(void *pCamHdl)
{
	camera_handle *pstCameraHandler = (camera_handle*)pCamHdl;
	if (pstCameraHandler) {
		if (ENST_CAMERA_STOP == pstCameraHandler->nWorkState)
			return 0;
		int nRet = hb_video_stop_stream(&pstCameraHandler->stHbVideoDev);
		pstCameraHandler->nWorkState = ENST_CAMERA_STOP;
		return nRet;
	}
	return -1;
}
int GetCameraInfo(void *pCamHdl, SomeCalibParam *pOutParam)
{
	camera_handle *pstCameraHandler = (camera_handle*)pCamHdl;
	if (pstCameraHandler) {
		if (TOFRET_SUCCESS == (TOFM_GetSomeCalibParam(pstCameraHandler->stDepthHandleCb[0].hTofMod, pOutParam)))
		{
			// PrintfSomeCalibParam(pOutParam);
			return 0;
		}
	}
	return -1;
}

void *OpenCamera(CAM_PARAM_S *pstCamParam)
{
	int iRet = 0;
	int i = 0;
	int iCamIndex = -1;
	unsigned int uiI2cSaveAddr;
	int iI2cDevId;
	PIPELINE_TYPE_E ePipelineType;
	CAM_TYPE_E eCamType;
	// V4L2_IMAGE_FMT_S stImageFmt;
	char acI2cDevPath[32];
	unsigned short au16SensorId[4] = {0};
	camera_handle *pstCameraHandler = NULL;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb_1;
	struct v4l2_device *vdev;

#ifdef HORIZON_PLATFORM
	HB_VIDEO_DEV_S *pstHbVideoDev;
	HB_VIN_VPSS_INIT_S *pstHbVinVpssInit;
	HB_SENSOR_INIT_S *pstHbSensorInit;
	MIPI_ATTR_S *pstMipiAttr;
#endif

	if (!pstCamParam)
	{
		printf("[%s] NULL ptr\n", __func__);
		return NULL;
	}

	pstCameraHandler = (camera_handle*)malloc(sizeof(camera_handle));
	if (!pstCameraHandler) 
	{
		printf("[%s] malloc camera_handle failed\n", __func__);
		return NULL;
	}
	memset(pstCameraHandler, 0, sizeof(*pstCameraHandler));

	eCamType = pstCamParam->eCamType;
	/* Fill camera info */
	if (eCamType == CAM_TYPE_TOF || eCamType == CAM_TYPE_TOF_RGBD)
	{
		pstCameraHandler->u32Width = RAW_WIDTH;
		pstCameraHandler->u32Height = RAW_HEIGHT;
		pstCameraHandler->u32PixelFormat = V4L2_PIX_FMT_SRGGB12;
		
		// stImageFmt.u32Width = RAW_WIDTH;
		// stImageFmt.u32Height= RAW_HEIGHT;
		// stImageFmt.u32PixelFormat = V4L2_PIX_FMT_SRGGB12;
	}
	else if (eCamType == CAM_TYPE_RGB)
	{
		pstCameraHandler->u32Width = RGB_WIDTH;
		pstCameraHandler->u32Height = RGB_HEIGHT;
		pstCameraHandler->u32PixelFormat = V4L2_PIX_FMT_NV12;
		
		// stImageFmt.u32Width = RGB_WIDTH;
		// stImageFmt.u32Height= RGB_HEIGHT;
		// stImageFmt.u32PixelFormat = V4L2_PIX_FMT_NV12;
	}
	else
	{
		printf("[%s] Unkown camera type: 0x%x, error exit!\n", __func__, eCamType);
		goto err_free;
	}
	
	pstCameraHandler->eCamType = eCamType;
	snprintf(pstCameraHandler->acDirection, 32, "%s", pstCamParam->acCamDirction);
	
#ifdef HORIZON_PLATFORM
	pstHbVideoDev = &pstCameraHandler->stHbVideoDev;
	memset(pstHbVideoDev, 0, sizeof(*pstHbVideoDev));
	pstHbVinVpssInit = &pstHbVideoDev->stHbVinVpssInit;
	pstHbSensorInit = &pstHbVideoDev->stHbSensorInit;
	pstMipiAttr = &pstHbVideoDev->stMipiAttr;

	if (eCamType == CAM_TYPE_TOF || eCamType == CAM_TYPE_TOF_RGBD)
	{
		memcpy(&pstHbVinVpssInit->stDevInfo, GetIrs2381cVinDevAttr(), sizeof(pstHbVinVpssInit->stDevInfo));
		memcpy(&pstHbVinVpssInit->stPipeInfo, GetIrs2381cVinPipeAttr(), sizeof(pstHbVinVpssInit->stPipeInfo));
		memcpy(&pstHbSensorInit->stMipiSensorInfo, GetIrs2381cSensorInfo(), sizeof(pstHbSensorInit->stMipiSensorInfo));
		memcpy(pstMipiAttr, GetIrs2381cMipiAttr(), sizeof(*pstMipiAttr));	
	
		pstHbVinVpssInit->pipeId = 1; 		/* ���������������ظ� */
		pstHbVinVpssInit->mipiIdx = pstHbSensorInit->stMipiSensorInfo.sensorInfo.entry_index;
		pstHbVinVpssInit->deseri_port = 0;	/* linearģʽVCͨ����Ĭ��Ϊ0 */
		pstHbVinVpssInit->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;
		

		pstHbSensorInit->devId = 1;
		pstHbSensorInit->mipiIdx = pstHbVinVpssInit->mipiIdx;
		pstHbSensorInit->sedres_index = 0;
		pstHbSensorInit->sedres_port = 0;
		pstHbSensorInit->bus_id = pstHbSensorInit->stMipiSensorInfo.sensorInfo.bus_num;
		pstHbSensorInit->port_id = pstHbSensorInit->stMipiSensorInfo.sensorInfo.port;

		pstHbVideoDev->devId = pstHbSensorInit->devId;
		pstHbVideoDev->pipeId = pstHbVinVpssInit->pipeId;
		pstHbVideoDev->mipiIdx = pstHbVinVpssInit->mipiIdx;
	}
	else if (eCamType == CAM_TYPE_RGB)
	{
		memcpy(&pstHbVinVpssInit->stDevInfo, GetGc2053VinDevAttr(), sizeof(pstHbVinVpssInit->stDevInfo));
		memcpy(&pstHbVinVpssInit->stPipeInfo, GetGc2053VinPipeAttr(), sizeof(pstHbVinVpssInit->stPipeInfo));
		memcpy(&pstHbSensorInit->stMipiSensorInfo, GetGc2053SensorInfo(), sizeof(pstHbSensorInit->stMipiSensorInfo));
		memcpy(pstMipiAttr, GetGc2053MipiAttr(), sizeof(*pstMipiAttr));	
	
		pstHbVinVpssInit->pipeId = 0; 		/* ���������������ظ� */
		pstHbVinVpssInit->mipiIdx = pstHbSensorInit->stMipiSensorInfo.sensorInfo.entry_index;
		pstHbVinVpssInit->deseri_port = 0;	/* linearģʽVCͨ����Ĭ��Ϊ0 */
		pstHbVinVpssInit->vin_vps_mode = VIN_OFFLINE_VPS_OFFINE;
		

		pstHbSensorInit->devId = 0;
		pstHbSensorInit->mipiIdx = pstHbVinVpssInit->mipiIdx;
		pstHbSensorInit->sedres_index = 0;
		pstHbSensorInit->sedres_port = 0;
		pstHbSensorInit->bus_id = pstHbSensorInit->stMipiSensorInfo.sensorInfo.bus_num;
		pstHbSensorInit->port_id = pstHbSensorInit->stMipiSensorInfo.sensorInfo.port;

		pstHbVideoDev->devId = pstHbSensorInit->devId;
		pstHbVideoDev->pipeId = pstHbVinVpssInit->pipeId;
		pstHbVideoDev->mipiIdx = pstHbVinVpssInit->mipiIdx;

		pstHbVideoDev->iNeedIsp = 1;
		pstHbVideoDev->iNeedInitVenc = 1;
	}

	iRet = hb_video_init(pstHbVideoDev);
	if (iRet)
	{
		printf("[%s] hb_video_init failed\n", __func__);
		goto err_free;
	}
	
#else
	/* V4L2 Init */
	iRet = GetVideoDevPath(pstCameraHandler->eCamType, pstCameraHandler->acDirection, &iCamIndex, &ePipelineType);
	if (iRet)
	{
		printf("[%s] Found 0 camera, type = 0x%x, dirction = %s\n", __func__, 
			pstCameraHandler->eCamType,
			pstCameraHandler->acDirection);
		goto err_free;
	}
	pstCameraHandler->camera_index = iCamIndex;
	pstCameraHandler->ePipelineType = ePipelineType;
	sprintf(pstCameraHandler->acDevPath, "%s%d", "/dev/video", pstCameraHandler->camera_index);

	printf("[%s] Found one camera, type = 0x%x, dirction = %s, camera_index = %d, ePipelineType = %u\n", __func__,
		pstCameraHandler->eCamType,
		pstCameraHandler->acDirection,
		pstCameraHandler->camera_index,
		pstCameraHandler->ePipelineType);
	
	iRet = v4l2_open(&pstCameraHandler->pstV4l2Dev, pstCameraHandler->acDevPath, &stImageFmt);
	if (iRet) 
	{
		printf("[%s] V4L2_Init camera[%d] failed\n", __func__, i);
		goto err_free;
	}
	vdev = pstCameraHandler->pstV4l2Dev;

	if (pstCamParam->isReqV4l2Buf)
	{
		/* V4L2 request buffers */
		iRet = v4l2_reqbufs(vdev, vdev->nbufs);
		if (iRet < 0) {
			printf("[%s] v4l2_reqbufs failed\n", __func__);
			goto err_v4l2_close;
		}

		/* Queue buffers to v4l2 domain and start streaming. */
		iRet = v4l2_qbuf(vdev);
		if (iRet < 0) {
			printf("[%s] v4l2_qbuf failed\n", __func__);
			goto err_v4l2_free;
		}

		/* Start V4L2 capturing now. */
		iRet = v4l2_start_capturing(vdev);
		if (iRet < 0) {
			printf("[%s] v4l2_start_capturing failed\n", __func__);
			goto err_v4l2_free;
		}
		vdev->is_streaming = 1;
	}
	
#endif

	/* I2C init */
	memset(&pstCameraHandler->stI2cCb, 0, sizeof(pstCameraHandler->stI2cCb));
	GetCamI2cSlaveAddr(pstCameraHandler->eCamType, pstCameraHandler->acDirection, &uiI2cSaveAddr);
	GetCamI2cDevId(pstCameraHandler->eCamType, pstCameraHandler->acDirection, &iI2cDevId);
	snprintf(acI2cDevPath, 32, "/dev/i2c-%d", iI2cDevId);
	
	iRet = I2cCbInit(&pstCameraHandler->stI2cCb, acI2cDevPath, uiI2cSaveAddr);
	if (iRet)
	{
		printf("I2cCbInit failed\n");
		goto err_video_uninit;
	}

	if (eCamType == CAM_TYPE_TOF || eCamType == CAM_TYPE_TOF_RGBD)
	{
		/* Get sensor ID */
		ReadSensorID(&pstCameraHandler->stI2cCb, au16SensorId);
		memcpy(pstCameraHandler->au16SensorId, au16SensorId, sizeof(au16SensorId));

		/* Read calibration data */	
		sprintf(pstCameraHandler->acCalibDataPath, "./calib-%04x-%04x-%04x-%04x.bin", 
						au16SensorId[0], au16SensorId[1], au16SensorId[2], au16SensorId[3]);
		iRet = ReadCalibData(&pstCameraHandler->stI2cCb, pstCameraHandler->acCalibDataPath, TOF_CALIB_OFFSET, MAX_READ_SZIE);
		if (iRet)
		{
			printf("read tof calib data failed\n");
			goto err_i2c_unit;
		}
	}

	if (eCamType == CAM_TYPE_TOF_RGBD)
	{
		sprintf(pstCameraHandler->acRgbdCalibDataPath, "./rgbd-calib-%04x-%04x-%04x-%04x.bin", 
						au16SensorId[0], au16SensorId[1], au16SensorId[2], au16SensorId[3]);
		iRet = ReadCalibData(&pstCameraHandler->stI2cCb, pstCameraHandler->acRgbdCalibDataPath, RGBD_CALIB_OFFSET, RGBD_CALIB_SIZE);
		if (iRet)
		{
			printf("read rgbd calib data failed\n");
			goto err_i2c_unit;
		}
	}

	if (eCamType == CAM_TYPE_TOF || eCamType == CAM_TYPE_TOF_RGBD)
	{
		/* Tof depth SDK init */
		TofDepthSdkGloableInit();
		
		pstDepthHandleCb = &pstCameraHandler->stDepthHandleCb[0];
		iRet = TofDepthSdkInit(pstDepthHandleCb, pstCameraHandler->acCalibDataPath, &pstCameraHandler->stI2cCb);
		if (iRet)
		{
			printf("TofDepthSdkInit 0 failed\n");
			goto err_i2c_unit;
		}

		if (pstCamParam->isDualDepthCalc)
		{
			pstCameraHandler->iDualDepthCalc = 1;
			pstDepthHandleCb_1 = &pstCameraHandler->stDepthHandleCb[1];
			iRet = TofDepthSdkInit(pstDepthHandleCb_1, pstCameraHandler->acCalibDataPath, &pstCameraHandler->stI2cCb);
			if (iRet)
			{
				printf("TofDepthSdkInit 1 failed\n");
				goto err_tof_sdk_uninit;
			}
		}
	}

	if (eCamType == CAM_TYPE_TOF_RGBD)
	{
	#ifdef RGBD
		/* RGBD SDK init */
		iRet = TofRgbdSdkInit(&pstCameraHandler->apstRgbdHandle[0], pstCameraHandler->acRgbdCalibDataPath);
		if (iRet)
		{
			printf("TofRgbdSdkInit 0 failed\n");
			goto err_tof_sdk_uninit;
		}
	#endif
	}

	return (void*)pstCameraHandler;

err_tof_sdk_uninit:
	TofDepthSdkUnInit(pstDepthHandleCb);
err_i2c_unit:
	I2cCbUninit(&pstCameraHandler->stI2cCb);

err_video_uninit:
#ifdef HORIZON_PLATFORM
	hb_video_Uninit(pstHbVideoDev);
#else
	v4l2_stop_capturing(vdev);
err_v4l2_free:
	v4l2_uninit_device(vdev);
	v4l2_reqbufs(vdev, 0);
err_v4l2_close:
	v4l2_close(pstCameraHandler->pstV4l2Dev);
#endif

err_free:
	free(pstCameraHandler);
	return NULL;		
}


void CloseCamera(void *pHandle)
{
	CAM_TYPE_E eCamType;
	camera_handle *pstCameraHandler;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb_1;

	if (!pHandle)
	{
		return;
	}

	pstCameraHandler = (camera_handle *)pHandle;
	eCamType = pstCameraHandler->eCamType;

	if (CAM_TYPE_TOF == eCamType || CAM_TYPE_TOF_RGBD == eCamType)
	{
		pstDepthHandleCb = &pstCameraHandler->stDepthHandleCb[0];
		pstDepthHandleCb_1 = &pstCameraHandler->stDepthHandleCb[1];

		TofDepthSdkUnInit(pstDepthHandleCb);
		pstDepthHandleCb->isExpTrdRunning = 0;
		pthread_join(pstDepthHandleCb->stTofExpPid, NULL);

		if (1 == pstCameraHandler->iDualDepthCalc)
		{
			TofDepthSdkUnInit(pstDepthHandleCb_1);
			pstDepthHandleCb_1->isExpTrdRunning = 0;
			pthread_join(pstDepthHandleCb_1->stTofExpPid, NULL);
		}
		
		TofDepthSdkGloableUnInit();
	}

	if (CAM_TYPE_TOF_RGBD == eCamType)
	{
		TofRgbdSdkUnit(pstCameraHandler->apstRgbdHandle[0]);
	}
	
	I2cCbUninit(&pstCameraHandler->stI2cCb);
	
	
#ifdef HORIZON_PLATFORM
	hb_video_Uninit(&pstCameraHandler->stHbVideoDev);
#else
	close_v4l2_device(pstCameraHandler->pstV4l2Dev);
#endif

	free(pHandle);
	pHandle = NULL;
}


int SetCameraFps(void *pHandle, unsigned int u32Fps)
{
	int iRet = 0;
	camera_handle *pstCameraHandler;

	pstCameraHandler = (camera_handle*)pHandle;

#ifdef HORIZON_PLATFORM
	iRet = hb_set_fps(&pstCameraHandler->stHbVideoDev, u32Fps);
	if (iRet)
	{
		printf("[%s] hb_set_fps failed\n", __func__);
		return iRet;
	}
#endif

	return iRet;
}

void yuv_to_rgb_table_init(void)
{
    int i;

    for(i=0; i<256; i++)
    {
        R1[i] = ((11530 * (i - 128)) >> 13);
        G1[i] = ((5873 * (i - 128)) >> 13);
        G2[i] = ((2830 * (i - 128)) >> 13);
        B1[i] = ((14574 * (i - 128)) >> 13);
    }
}

void yuv_to_rgb(int y, int u, int v, int *r, int *g, int *b)
{
    *r = (int)(y + R1[v]);
    *r = (*r < 0) ? 0 : ((*r > 255) ? 255 : *r);

    *g = (int)(y - G1[v] - G2[u]);
    *g = (*g < 0) ? 0 : ((*g > 255) ? 255 : *g);

    *b = (int)(y + B1[u]);
    *b = (*b < 0) ? 0 : ((*b > 255) ? 255 : *b);
}


void nv12_to_bgr888_buffer(unsigned char *yuv, unsigned char *rgb, int width, int height)
{
    int i, j;
	int y, u, v;
    int r, g, b;
	unsigned char *Y;
	unsigned char *UV;

	Y = yuv;
	UV = yuv + width*height;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			y = Y[i * width + j];
			u = UV[(i/2 * width + j/2 * 2)];
			v = UV[(i/2 * width + j/2 * 2) + 1];
			
			yuv_to_rgb(y, u, v, &r, &g, &b);
			
			rgb[(i * width + j) * 3 + 0] = (unsigned char)b;
			rgb[(i * width + j) * 3 + 1] = (unsigned char)g;
			rgb[(i * width + j) * 3 + 2] = (unsigned char)r;
		}
	}
}


