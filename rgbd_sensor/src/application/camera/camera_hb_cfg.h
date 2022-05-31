#ifndef __CAMERA_HB_CFG_H__
#define __CAMERA_HB_CFG_H__


#include "camera_control.h"
#include "hb_mipi_api.h"
#include "hb_vin_api.h"


#define PHASE_NUM 				(17)
#define RAW_WIDTH           	(224)
#define PHASE_RAW_HEIGHT        (129)
#define RAW_HEIGHT				(PHASE_RAW_HEIGHT*PHASE_NUM)
#define RAW_SIZE				(RAW_WIDTH*RAW_HEIGHT*2)
#define PIXEL_FORMAT   	 		V4L2_PIX_FMT_SBGGR12
#define CALIB_ROI				(129)
#define ACTIVE_HEIGHT			(108)

#define CALIB_RAW_SIZE			(RAW_WIDTH*CALIB_ROI*PHASE_NUM*2)
#define DEPTH_DATA_SIZE			(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(float))
#define POINTCLOUD_SIZE			(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(float)*3)
#define GREY_DATA_SIZE			(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(float))
#define GREY_U16_DATA_SIZE		(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(unsigned short))
#define CONFIDENCE_SIZE	 		(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(unsigned short))	
#define NOISE_SIZE				(RAW_WIDTH*ACTIVE_HEIGHT*sizeof(float))

#define RGB_WIDTH				(1920)
#define RGB_HEIGHT				(1080)
#define RGB_SIZE				(RGB_WIDTH*RGB_HEIGHT*3)
#define RGB_YUV_SZIE			(RGB_WIDTH*RGB_HEIGHT*3/2)
#define RGB_ENC_SIZE			(500*1024)

#define I2C_TOF_SLAVE_ADDR		(0x3d)
#define I2C_RGB_SLAVE_ADDR		(0x37)
#define I2C_TOF_DEV_ID			(2)
#define I2C_RGB_DEV_ID			(2)

#define SELECT_MODULE_NAME		"T00P11A"
#define SELECT_MODULE_CFG_FILE	"./parameter/T00P11A-17.ini"
#define EXP_MAX_AEF				(50)
#define EXP_MAX_FEF				(720)
#define SELECT_MODULE_GUEST_ID	MODULE_GUEST_ID_DEF


int GetVideoDevPath(CAM_TYPE_E eCamType, char *pcCamDirection, int *piCamDevId, PIPELINE_TYPE_E *pePipeType);
int GetCamI2cSlaveAddr(CAM_TYPE_E eCamType, char *pcCamDirection, unsigned int *puiSlaveAddr);
int GetCamI2cDevId(CAM_TYPE_E eCamType, char *pcCamDirection, int *piI2cDevId);

VIN_DEV_ATTR_S *GetIrs2381cVinDevAttr(void);
VIN_PIPE_ATTR_S *GetIrs2381cVinPipeAttr(void);
MIPI_SENSOR_INFO_S *GetIrs2381cSensorInfo(void);
MIPI_ATTR_S *GetIrs2381cMipiAttr(void);
VIN_DEV_ATTR_S *GetGc2053VinDevAttr(void);
VIN_PIPE_ATTR_S *GetGc2053VinPipeAttr(void);
MIPI_SENSOR_INFO_S *GetGc2053SensorInfo(void);
MIPI_ATTR_S *GetGc2053MipiAttr(void);


#endif

