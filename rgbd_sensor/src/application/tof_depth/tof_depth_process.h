#ifndef __TOF_DEPTH_PROCESS_H__
#define __TOF_DEPTH_PROCESS_H__


#include <string>

#include "tof_mod_sdk.h"
#include "tof_error.h"
#include "typedef.h"
#include "camera_control.h"


#define MODULE_ID_LEN 			(4)//模组ID长度
#define CALIB_DATA_MAGIC 		0x594E5553

#define RINS_FILE_SIZE 			(1024)
#define RGBD_CALIB_SIZE			(1024)
#define	MAX_READ_PAGE	 		(8*1024)
#define	MAX_READ_SZIE	 		(16*8*1024+RINS_FILE_SIZE)

#define START_SETUP_EXP 		1
#define STOP_SETUP_EXP  		0


typedef struct tagcalib_frame_header_v20_t
{
	unsigned int magic;
	/* magic number, 0x53 0x55 0x4E 0x59 */
	unsigned int dev_type;
	/* to identify different devices,self-define one enum structure */
	unsigned int total_size;
	/* total calib data size in bytes */
	unsigned int frame_size;
	/* one frame calib data size */
	unsigned int frame_idx;
	/* frame index */
	unsigned short module_id[MODULE_ID_LEN];
	/* module id */
	unsigned int rsv;
	/* reserved for future use */
}calib_frame_header_v20_t;


typedef struct tof_depth_data_info
{
	UINT64  timeStamp;//时间戳
	UINT32  uiFrameCnt;//帧计数
	UINT32  frameWidth;//深度图宽度
	UINT32  frameHeight;//深度图高度
	PointData *pfPointData;//点云数据
	GRAY_FORMAT grayFormat;//pGrayData内数据格式
	void   *pGrayData;//灰度数据
	UINT16   *pu16GrayData;//u16灰度数据
	UINT16	*pu16Confidence;//置信度
	float	*pfNoise;//噪声
	FLOAT32* pDepthData;//射线距离
} TOF_DEPTH_DATA_INFO_S;


void TofDepthSdkGloableInit();
void TofDepthSdkGloableUnInit();
int TofDepthSdkInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb, char *pcCalibDataPath, void *pHalUserData);
int TofDepthSdkUnInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb);
int TofDepthProcess(HTOFM hTofMod, IMAGE_DATA_INFO_S *pstTofRawDataInfo, TOF_DEPTH_DATA_INFO_S *pstTofDepthDataInfo);
int TofDepthProcessExp(void *pCamHandle, IMAGE_DATA_INFO_S *pstTofRawDataInfo, TOF_DEPTH_DATA_INFO_S *pstTofDepthDataInfo, int iDepthHandleIndex);
int CaculateGrayPixelBytes(const GRAY_FORMAT format);
void HandleDepthData(const UINT32 threadIndex, UINT32 frameIndex, std::string& strSaveDir, std::string strTofName, TOF_DEPTH_DATA_INFO_S* tofFrameData);
void *Sunny_SetTofEXP(void *para);

bool TofGray2U8(void* pGrayData, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, void ** pOutGrayU8);

#ifdef RGBD
int TofRgbdSdkInit(HTOFRGBD *ppstRgbdHandle, const char *pcRgbdCalibPath);
int TofRgbdSdkUnit(HTOFRGBD pstRgbdHandle);
int TofRgbdProcess(HTOFRGBD pstRgbdHandle, TofRgbdInputData* pDataIn, TofRgbdOutputData* pDataOut);
void HandleTofRgbdOutputData(unsigned int frameIndex, const std::string& strSaveDir, TofRgbdInputData* pDataIn, TofRgbdOutputData* rgbdData);
#endif

#endif

