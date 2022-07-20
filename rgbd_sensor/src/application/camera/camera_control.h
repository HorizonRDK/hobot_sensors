#ifndef	__CAMERA_CONTROL_H__
#define __CAMERA_CONTROL_H__


#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>

#include "tof_i2c.h"
#include "typedef.h"
#include "tof_mod_sdk.h"
#ifdef RGBD
#include "tof_rgbd_sdk.h"
#endif
// #include "v4l2_capture.h"

#ifdef HORIZON_PLATFORM
#include "horizon_mpp.h"
#endif


typedef enum tagCamType
{
	CAM_TYPE_RGB = 0x01,
	CAM_TYPE_TOF,
	CAM_TYPE_TOF_RGBD,
} CAM_TYPE_E;

typedef struct tagCamParam
{
	CAM_TYPE_E eCamType;
	int isDualDepthCalc;
	int isReqV4l2Buf;
	char acCamDirction[32];
} CAM_PARAM_S;

typedef enum tof_exp_mode {
	SINGLE_FRAME_MODE,
	LONG_SHORT_FRAME_MODE,
} TOF_EXP_MODE_E;

typedef struct set_tof_exp{
  HTOFM hTofMod;
  TOF_EXP_MODE_E eExpMode;
  UINT32 expTime;
  UINT32 expTime_AEF;
  UINT32 expTime_FEF;
  UINT32 flag;
}Tof_SetExp;

typedef enum PipelineType
{
	PIPELINE_CIF = 0,
	PIPELINE_ISP,
} PIPELINE_TYPE_E;

typedef struct tagDepthHandleCb
{
	HTOFM hTofMod;
	unsigned int uiSupportedTofFilter;
	int isExpTrdRunning;
	pthread_t stTofExpPid;
	Tof_SetExp stTofSetExp;
} DEPTH_HANDLE_CB_S;

typedef struct camera_hal
{
    int camera_index;
	CAM_TYPE_E eCamType;
	PIPELINE_TYPE_E ePipelineType;
	unsigned int u32Width;
	unsigned int u32Height;
	unsigned int u32PixelFormat;
	int iDualDepthCalc;
	I2C_CB_S stI2cCb;
	unsigned short au16SensorId[4];
	char acDevPath[32];
	char acDirection[32];
	char acCalibDataPath[64];
	char acRgbdCalibDataPath[64];
#ifdef HORIZON_PLATFORM
	HB_VIDEO_DEV_S stHbVideoDev;
#else
	struct v4l2_device *pstV4l2Dev;
#endif
	DEPTH_HANDLE_CB_S stDepthHandleCb[4];
#ifdef RGBD
	HTOFRGBD apstRgbdHandle[4];
#endif
} camera_handle;


void *OpenCamera(CAM_PARAM_S *pstCamParam);
void CloseCamera(void *pHandle);

int CameraStreamON(void *pCamHandle);
int CameraStreamOFF(void *pCamHandle);

int SetCameraFps(void *pHandle, unsigned int u32Fps);

int GetRawData(void *pHandle, RAW_DATA_INFO_S *pstRawDataInfo);
int GetImageData(void *pHandle, IMAGE_DATA_INFO_S *pstImageDataInfo);

int FrameEncode(void *pHandle, IMAGE_DATA_INFO_S *pstImageInput, IMAGE_DATA_INFO_S *pstImageOutput);

int GetSensorID(void *pCamHandle, unsigned short au16SensorID[4]);
unsigned int GetSupportTofFilter(void *pCamHandle);
int GetTofFilterStatus(void *pCamHandle, unsigned int *puiEnableTypeList, unsigned int *puiDisableTypeList);
int SetTofFilterStatus(void *pCamHandle, unsigned int *puiEnableTypeList, unsigned int *puiDisableTypeList);

void yuv_to_rgb_table_init(void);
void nv12_to_bgr888_buffer(unsigned char *yuv, unsigned char *rgb, int width, int height);


#endif	/* __CAMERA_CONTROL_H__ */

