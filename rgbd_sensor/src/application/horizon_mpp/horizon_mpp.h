#ifndef __HORIZON_MPP_H__
#define __HORIZON_MPP_H__

#include "hb_mipi_api.h"
#include "hb_vin_api.h"
#include "hb_vps_api.h"

#include "comm_typedef.h"


typedef struct tagHbVinVpssInit
{
	int pipeId;	/* 一路pipe链路，ID不能重复 */
	uint32_t mipiIdx;
	uint32_t deseri_port;	/* VC通道ID */
	uint32_t vin_vps_mode;
	VIN_DEV_ATTR_S stDevInfo;
	VIN_PIPE_ATTR_S stPipeInfo;
} HB_VIN_VPSS_INIT_S;

typedef struct tagHbSensorInit
{
	int devId;
	int mipiIdx;
	int bus_id;
	int port_id;
	int sedres_index;
	int sedres_port;
	MIPI_SENSOR_INFO_S stMipiSensorInfo;
} HB_SENSOR_INIT_S;

typedef struct tagHbMemBlock
{
	int isAvailable;
	uint64_t mmz_paddr;
	char* mmz_vaddr;
} HB_MEM_BLOCK_S;

typedef struct tagHbVideoDev
{
	int isInit;
	int vin_fd;
	int pipeId;
	int devId;
	uint32_t mipiIdx;
	int VeChn;
	int is_streaming;
	int iNeedIsp;
	int iNeedInitVps;
	int iNeedInitVenc;
	HB_MEM_BLOCK_S stMemBlock;
	HB_VIN_VPSS_INIT_S stHbVinVpssInit;
	HB_SENSOR_INIT_S stHbSensorInit;
	MIPI_ATTR_S stMipiAttr;
} HB_VIDEO_DEV_S;


int hb_video_init(HB_VIDEO_DEV_S *pstHbVideoDev);
int hb_video_Uninit(HB_VIDEO_DEV_S *pstHbVideoDev);

int hb_video_start_stream(HB_VIDEO_DEV_S *pstHbVideoDev);
int hb_video_stop_stream(HB_VIDEO_DEV_S *pstHbVideoDev);

int hb_get_raw_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo);
int hb_get_yuv_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo);

int hb_encode_send_frame(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo);
int hb_get_encode_data(HB_VIDEO_DEV_S *pstHbVideoDev, IMAGE_DATA_INFO_S *pstImageDataInfo);

int hb_set_fps(HB_VIDEO_DEV_S *pstHbVideoDev, unsigned int u32Fps);


#endif

