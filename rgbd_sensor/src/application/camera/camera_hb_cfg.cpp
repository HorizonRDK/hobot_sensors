#include "camera_hb_cfg.h"


static VIN_DEV_ATTR_S DEV_ATTR_IRS2381C_LINEAR_BASE;
static VIN_PIPE_ATTR_S PIPE_ATTR_IRS2381C_LINEAR_BASE;
static MIPI_SENSOR_INFO_S SENSOR_2LANE_IRS2381C_10FPS_12BIT_LINEAR_INFO;
static MIPI_ATTR_S MIPI_2LANE_SENSOR_IRS2381C_10FPS_12BIT_LINEAR_ATTR;

static VIN_DEV_ATTR_S DEV_ATTR_GC2053_LINEAR_BASE;
static VIN_PIPE_ATTR_S PIPE_ATTR_GC2053_LINEAR_BASE;
static MIPI_SENSOR_INFO_S SENSOR_2LANE_GC2053_10FPS_10BIT_LINEAR_INFO;
static MIPI_ATTR_S MIPI_2LANE_SENSOR_GC2053_10FPS_10BIT_LINEAR_ATTR;


static bool is_v4l2_mainpath(char *buf)
{
    if (strstr(buf, "mainpath"))
        return true;
    else
        return false;
}

static bool is_v4l2_cif(char *buf)
{
    if (strstr(buf, "stream_cif_mipi_id0"))
        return true;
    else
        return false;
}

int GetVideoDevPath(CAM_TYPE_E eCamType, char *pcCamDirection, int *piCamDevId, PIPELINE_TYPE_E *pePipeType)
{
	FILE *fp = NULL;
    char buf[1024];
    int i;
	int index = -1;
	int iRet = 0;
	PIPELINE_TYPE_E ePipelineType;
    char cmd[128];

	if (!pcCamDirection || !piCamDevId || !pePipeType)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

    for (i = 0; i < 40; i++)
    {
        snprintf(cmd, sizeof(cmd), "/sys/class/video4linux/video%d/name", i);
        if (access(cmd, F_OK))
            continue;
        snprintf(cmd, sizeof(cmd), "cat /sys/class/video4linux/video%d/name", i);
        fp = popen(cmd, "r");
        if (fp)
        {
            if (fgets(buf, sizeof(buf), fp))
            {
				if (is_v4l2_cif(buf) && ((CAM_TYPE_TOF == eCamType) || (CAM_TYPE_TOF_RGBD == eCamType)))
				{
                	index = i;
					ePipelineType = PIPELINE_CIF;
					break;
				}

				if (is_v4l2_mainpath(buf) && (CAM_TYPE_RGB == eCamType))
				{
                	index = i;
					ePipelineType = PIPELINE_ISP;
					break;
				}
            }
            pclose(fp);
        }
    }

    if (index < 0)
    {
        printf("Found 0 v4l2 device, please configure sensor driver...\n");
		*piCamDevId = -1;
        return -1;
    }

	*piCamDevId = index;
	*pePipeType = ePipelineType;

	return 0;
}

int GetCamI2cSlaveAddr(CAM_TYPE_E eCamType, char *pcCamDirection, unsigned int *puiSlaveAddr)
{
	if (!pcCamDirection || !puiSlaveAddr)
	{
		printf("[%s] NULL ptr\n", __func__);
		return -1;
	}

	if ((CAM_TYPE_TOF == eCamType) || (CAM_TYPE_TOF_RGBD == eCamType))
	{
		*puiSlaveAddr = I2C_TOF_SLAVE_ADDR;
	}
	else if (CAM_TYPE_RGB == eCamType)
	{
		*puiSlaveAddr = I2C_RGB_SLAVE_ADDR;
	}
	else
	{
		return -1;
	}

	return 0;
}

int GetCamI2cDevId(CAM_TYPE_E eCamType, char *pcCamDirection, int *piI2cDevId)
{
	unsigned int uiSlaveAddr;

	if (!pcCamDirection || !piI2cDevId)
	{
		printf("[%s] NULL ptr\n", __func__);
		return -1;
	}

	if ((CAM_TYPE_TOF == eCamType) || (CAM_TYPE_TOF_RGBD == eCamType))
	{
		*piI2cDevId = I2C_TOF_DEV_ID;
	}
	else if (CAM_TYPE_RGB == eCamType)
	{
		*piI2cDevId = I2C_RGB_DEV_ID;
	}
	else
	{
		return -1;
	}

	return 0;
}

VIN_DEV_ATTR_S *GetIrs2381cVinDevAttr(void)
{
	VIN_DEV_ATTR_S *pstVinDevAttr = &DEV_ATTR_IRS2381C_LINEAR_BASE;
	
	VIN_DEV_SIZE_S *pstSize = &pstVinDevAttr->stSize;
	VIN_MIPI_ATTR_S *pstMipiAttr = &pstVinDevAttr->mipiAttr;
	VIN_DEV_INPUT_DDR_ATTR_S *pstInputDdrAttr = &pstVinDevAttr->DdrIspAttr;
	VIN_DEV_OUTPUT_DDR_S *pstOutDdr = &pstVinDevAttr->outDdrAttr;
	VIN_DEV_OUTPUT_ISP_S *pstOutIsp = &pstVinDevAttr->outIspAttr;

	memset(pstVinDevAttr, 0, sizeof(*pstVinDevAttr));
	
	pstSize->format = 0;	// 像素格式， format 为 0 代表是raw, 根据 pixel_lenght 来表示究竟是 raw8, raw12 还是 raw16 。
	pstSize->width = RAW_WIDTH;	// 数据宽
	pstSize->height = RAW_HEIGHT;	// 数据高
	pstSize->pix_length = 2;	// format=0表示raw    pix_length= 0-raw8   1-raw10  2-raw12
	
	pstMipiAttr->enable = 1; // mipi 使能 ,0 是关闭， 1 是使能
	pstMipiAttr->ipi_channels = 1; // sif和isp之间的硬件通道，可以把多个mipi虚拟通道合并到这一个ipi channel中，ipi_channels 表示用了几个 channel ，默认是 0 开始，如果设置是 2 ，是用了 0 ，1
	pstMipiAttr->ipi_mode = 1; // 当 DOL2 分成两路 linear 或者 DOL3 分成一路 DOl2 和一路 linear 或者三路linear 的时候，此值就赋值为 2 或 3.
	pstMipiAttr->enable_mux_out = 1; // 未用
	pstMipiAttr->enable_frame_id = 1; // 是否使能 frameid, 是否做智能的时候要用？
	pstMipiAttr->enable_bypass = 0; // 使能mipi host到mipi dev的bypass
	pstMipiAttr->enable_line_shift = 0; // 未用
	pstMipiAttr->enable_id_decoder = 0; // 未用
	pstMipiAttr->set_init_frame_id = 1; // 初始 frame id 值一般为 1
	pstMipiAttr->set_line_shift_count = 0; // 未用
	pstMipiAttr->set_bypass_channels = 1; // 未用
	pstMipiAttr->enable_pattern = 0; // 是否使能 testpartern

	pstInputDdrAttr->buf_num = 4; // 回灌的存储数据的 buf 数目
	pstInputDdrAttr->raw_feedback_en = 0; // 使能回灌模式，不能和 offline 模式同时开启，独立使用
	pstInputDdrAttr->data.format = 0;
	pstInputDdrAttr->data.width = RAW_WIDTH;
	pstInputDdrAttr->data.height = RAW_HEIGHT;
	pstInputDdrAttr->data.pix_length = 1;

	pstOutDdr->stride = RAW_WIDTH*1.5; // 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来（而且结果要32对齐），8bit：x1, 10bit: x1.25 12bit: x1.5,例F37 raw10，1920 x 1.25 = 2400
	pstOutDdr->buffer_num = 8; // dev 输出到 ddr 的 buf 个数，最少可设置为6

	pstOutIsp->dol_exp_num = 1; // 曝光模式， 1 为普通模式， dol 2 或者 3设置对应数目
	pstOutIsp->enable_dgain = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_short = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_medium = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_long = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->vc_short_seq = 0; // 用来描述 DOL2/3 模式下，短帧的顺序
	pstOutIsp->vc_medium_seq = 0; // 用来描述 DOL2/3 模式下，普通帧的顺序
	pstOutIsp->vc_long_seq = 0; // 用来描述 DOL2/3 模式下，长帧的顺序

	return pstVinDevAttr;
}


VIN_PIPE_ATTR_S *GetIrs2381cVinPipeAttr(void)
{
	VIN_PIPE_ATTR_S *pstVinDevAttr = &PIPE_ATTR_IRS2381C_LINEAR_BASE;
	
	memset(pstVinDevAttr, 0, sizeof(*pstVinDevAttr));

	pstVinDevAttr->ddrOutBufNum = 8; // ISP->GDC ddr的buf数量，最少可以设置成 2
	pstVinDevAttr->frameDepth = 8;
	pstVinDevAttr->snsMode = SENSOR_NORMAL_MODE; // sensor 工作模式，要和MIPI_SENSOR_INFO_S配置的sensor_mode一致
	pstVinDevAttr->stSize.format = 0;
	pstVinDevAttr->stSize.width = RAW_WIDTH;
	pstVinDevAttr->stSize.height = RAW_HEIGHT;

	pstVinDevAttr->cfaPattern = PIPE_BAYER_BGGR; // 数据格式布局(sensor输出的图像格式是RGGB还是BGGR),和sensor保持一致，查sensor数据手册可知
	pstVinDevAttr->temperMode = 2; // 时序降噪模式，2代表2帧融合，3代表3帧融合
	pstVinDevAttr->ispBypassEn = 0; // 是否使能 isp 的 bypass, sensor必须支持输出yuv
	pstVinDevAttr->ispAlgoState = 0; // 是否启动 3a 算法库 ,1 是启动， 0 是关闭
	pstVinDevAttr->bitwidth = 12; // raw图位宽，有效值 8 、 10 、 12 、14 、 16 、 20
	pstVinDevAttr->calib.mode = 0; // 是否开启 sensor 矫正数据加载，1 是开启，0 是关闭。ISP使用这些数据
	pstVinDevAttr->calib.lname = (char*)"libirs2381c.so"; // 对应使用的校准库

	return pstVinDevAttr;
}


MIPI_SENSOR_INFO_S *GetIrs2381cSensorInfo(void)
{
	MIPI_SENSOR_INFO_S *pstSensorInfo = &SENSOR_2LANE_IRS2381C_10FPS_12BIT_LINEAR_INFO;
	MIPI_SNS_INFO_S *pstSnsInfo = &pstSensorInfo->sensorInfo;
	
	memset(pstSensorInfo, 0, sizeof(*pstSensorInfo));

	pstSensorInfo->deseEnable = 0; // 该 sensor 是否有 serdes（串解器）
	pstSensorInfo->inputMode = INPUT_MODE_MIPI; // sensor 接入方式,mipi还是dvp
	pstSnsInfo->port = 1; // sensor的逻辑编号，必须从0开始
	pstSnsInfo->dev_port = 0; // 每路 sensor 操作的驱动节点，一个驱动支持多个节点。 snsinfo 中的dev_port 必须等于pipeId，多目摄像头设置的时候需要特别注意
	pstSnsInfo->bus_type = 0; // 访问总线类型， 0 是 i2c,1 是 spi
	pstSnsInfo->bus_num = 2; // 总线号，根据具体板子硬件原理图确定 , 不配置默认 i2c5
	pstSnsInfo->fps = 10; // 帧率，用来选择使用哪一组帧率的sensor参数
	pstSnsInfo->resolution = RAW_HEIGHT; // sensor 行数, 必须要和mipi属性配置一致
	pstSnsInfo->sensor_addr = 0x3d; // sensor i2c 设备地址
	pstSnsInfo->entry_index = 1; // sensor 使用的 mipi 索引, 0~3，对应mipi host的序号
	pstSnsInfo->sensor_mode = NORMAL_M; // sensor 工作模式， 1 是 normal,2 是dol2,3 是 dol3
	pstSnsInfo->reg_width = 16; // sensor 寄存器地址宽度
	pstSnsInfo->sensor_name = (char*)"irs2381c"; // sensor的名字，在libcam.so中会根据这个名字组合出 libf37.so
											// 这个库文件，然后调用dlopen运行时打开sensor库， libf37.so 这个库的生成
											// 请参考sensor点亮文档，以f37 为例是 f37_utility.c f37_utility.h
											// 请放到sdk包的hbre/camera/utility/sensor目录下make生成libf37.so文件

	return pstSensorInfo;
}


MIPI_ATTR_S *GetIrs2381cMipiAttr(void)
{
	MIPI_ATTR_S *pstMipiAttr = &MIPI_2LANE_SENSOR_IRS2381C_10FPS_12BIT_LINEAR_ATTR;
	MIPI_HOST_CFG_S *pstHostCfg = &pstMipiAttr->mipi_host_cfg;
	
	memset(pstMipiAttr, 0, sizeof(*pstMipiAttr));

	pstHostCfg->lane = 2; // 硬件上sensor用了几个mipi数据lane，f37用的1 lane, os8a10 4K的用了4 lane
	pstHostCfg->datatype = 0x2c; // sensor的输出数据类型，请参考 《X3J3平台AIOT媒体系统接口手册.pdf》第3.5.36节 DATA TYPE
	pstHostCfg->mclk = 2400; // mipi 模块主时钟，目前默认是24MHz
	pstHostCfg->mipiclk = 384; // sensor 输出总的 mipibit rate, 单位Mbits/秒
	pstHostCfg->fps = 10; // sensor 输出实际帧率, 和 MIPI_SENSOR_INFO_S 是什么关系？
	pstHostCfg->width = RAW_WIDTH; // sensor实际输出的分辨率，一般根据sensor供应商的配置参数配置就行
	pstHostCfg->height = RAW_HEIGHT;
	pstHostCfg->linelenth = 317; // sensor 输出带 blanking 的总行长， 一般根据sensor供应商的配置参数配置就行 2496
	pstHostCfg->framelenth = 15664; // sensor 输出带 blanking的总行数 13080
	pstHostCfg->settle = 67; // sensor 输出实际Ttx-zero + Ttx-prepare时间（ clk 为单位）, 一般0-100之间尝试 100
	pstHostCfg->channel_num = 1; // 使用虚通道的个数, linear 用一个，dol2 用 2 个， dol3 用3个
	//pstHostCfg->channel_sel = {0}; // 保存每个虚通道的值，在调用HB_VIN_SetDevVCNumber接口时用第一个，调用HB_VIN_AddDevVCNumber用后面的2 3 4的配置

	pstMipiAttr->dev_enable = 0; // mipi dev 是否使能， 1是使能， 0 是关闭, 只用开启mipi bypass的开启

	return pstMipiAttr;
}


/* gc2053 */
VIN_DEV_ATTR_S *GetGc2053VinDevAttr(void)
{
	VIN_DEV_ATTR_S *pstVinDevAttr = &DEV_ATTR_GC2053_LINEAR_BASE;
	
	VIN_DEV_SIZE_S *pstSize = &pstVinDevAttr->stSize;
	VIN_MIPI_ATTR_S *pstMipiAttr = &pstVinDevAttr->mipiAttr;
	VIN_DEV_INPUT_DDR_ATTR_S *pstInputDdrAttr = &pstVinDevAttr->DdrIspAttr;
	VIN_DEV_OUTPUT_DDR_S *pstOutDdr = &pstVinDevAttr->outDdrAttr;
	VIN_DEV_OUTPUT_ISP_S *pstOutIsp = &pstVinDevAttr->outIspAttr;

	memset(pstVinDevAttr, 0, sizeof(*pstVinDevAttr));
	
	pstSize->format = 0;	// 像素格式， format 为 0 代表是raw, 根据 pixel_lenght 来表示究竟是 raw8, raw12 还是 raw16 。
	pstSize->width = RGB_WIDTH;	// 数据宽
	pstSize->height = RGB_HEIGHT;	// 数据高
	pstSize->pix_length = 1;	// format=0表示raw    pix_length= 0-raw8   1-raw10  2-raw12
	
	pstMipiAttr->enable = 1; // mipi 使能 ,0 是关闭， 1 是使能
	pstMipiAttr->ipi_channels = 1; // sif和isp之间的硬件通道，可以把多个mipi虚拟通道合并到这一个ipi channel中，ipi_channels 表示用了几个 channel ，默认是 0 开始，如果设置是 2 ，是用了 0 ，1
	pstMipiAttr->ipi_mode = 1; // 当 DOL2 分成两路 linear 或者 DOL3 分成一路 DOl2 和一路 linear 或者三路linear 的时候，此值就赋值为 2 或 3.
	pstMipiAttr->enable_mux_out = 1; // 未用
	pstMipiAttr->enable_frame_id = 1; // 是否使能 frameid, 是否做智能的时候要用？
	pstMipiAttr->enable_bypass = 0; // 使能mipi host到mipi dev的bypass
	pstMipiAttr->enable_line_shift = 0; // 未用
	pstMipiAttr->enable_id_decoder = 0; // 未用
	pstMipiAttr->set_init_frame_id = 1; // 初始 frame id 值一般为 1
	pstMipiAttr->set_line_shift_count = 0; // 未用
	pstMipiAttr->set_bypass_channels = 1; // 未用
	pstMipiAttr->enable_pattern = 0; // 是否使能 testpartern

	pstInputDdrAttr->buf_num = 4; // 回灌的存储数据的 buf 数目
	pstInputDdrAttr->raw_feedback_en = 0; // 使能回灌模式，不能和 offline 模式同时开启，独立使用
	pstInputDdrAttr->data.format = 0;
	pstInputDdrAttr->data.width = RGB_WIDTH;
	pstInputDdrAttr->data.height = RGB_HEIGHT;
	pstInputDdrAttr->data.pix_length = 1;

	pstOutDdr->stride = RGB_WIDTH*1.25; // 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来（而且结果要32对齐），8bit：x1, 10bit: x1.25 12bit: x1.5,例F37 raw10，1920 x 1.25 = 2400
	pstOutDdr->buffer_num = 8; // dev 输出到 ddr 的 buf 个数，最少可设置为6

	pstOutIsp->dol_exp_num = 1; // 曝光模式， 1 为普通模式， dol 2 或者 3设置对应数目
	pstOutIsp->enable_dgain = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_short = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_medium = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->set_dgain_long = 0; // ISP 内部调试参数，暂可忽略
	pstOutIsp->vc_short_seq = 0; // 用来描述 DOL2/3 模式下，短帧的顺序
	pstOutIsp->vc_medium_seq = 0; // 用来描述 DOL2/3 模式下，普通帧的顺序
	pstOutIsp->vc_long_seq = 0; // 用来描述 DOL2/3 模式下，长帧的顺序

	return pstVinDevAttr;
}


VIN_PIPE_ATTR_S *GetGc2053VinPipeAttr(void)
{
	VIN_PIPE_ATTR_S *pstVinDevAttr = &PIPE_ATTR_GC2053_LINEAR_BASE;
	
	memset(pstVinDevAttr, 0, sizeof(*pstVinDevAttr));

	pstVinDevAttr->ddrOutBufNum = 8; // ISP->GDC ddr的buf数量，最少可以设置成 2
	pstVinDevAttr->frameDepth = 8;
	pstVinDevAttr->snsMode = SENSOR_NORMAL_MODE; // sensor 工作模式，要和MIPI_SENSOR_INFO_S配置的sensor_mode一致
	pstVinDevAttr->stSize.format = 0;
	pstVinDevAttr->stSize.width = RGB_WIDTH;
	pstVinDevAttr->stSize.height = RGB_HEIGHT;

	pstVinDevAttr->cfaPattern = PIPE_BAYER_RGGB; // 数据格式布局(sensor输出的图像格式是RGGB还是BGGR),和sensor保持一致，查sensor数据手册可知
	pstVinDevAttr->temperMode = 2; // 时序降噪模式，2代表2帧融合，3代表3帧融合
	pstVinDevAttr->ispBypassEn = 0; // 是否使能 isp 的 bypass, sensor必须支持输出yuv
	pstVinDevAttr->ispAlgoState = 1; // 是否启动 3a 算法库 ,1 是启动， 0 是关闭
	pstVinDevAttr->bitwidth = 10; // raw图位宽，有效值 8 、 10 、 12 、14 、 16 、 20
	pstVinDevAttr->calib.mode = 1; // 是否开启 sensor 矫正数据加载，1 是开启，0 是关闭。ISP使用这些数据
	pstVinDevAttr->calib.lname = (char*)"/lib/sensorlib/libgc2053_linear.so"; // 对应使用的校准库

	return pstVinDevAttr;
}


MIPI_SENSOR_INFO_S *GetGc2053SensorInfo(void)
{
	MIPI_SENSOR_INFO_S *pstSensorInfo = &SENSOR_2LANE_GC2053_10FPS_10BIT_LINEAR_INFO;
	MIPI_SNS_INFO_S *pstSnsInfo = &pstSensorInfo->sensorInfo;
	
	memset(pstSensorInfo, 0, sizeof(*pstSensorInfo));

	pstSensorInfo->deseEnable = 0; // 该 sensor 是否有 serdes（串解器）
	pstSensorInfo->inputMode = INPUT_MODE_MIPI; // sensor 接入方式,mipi还是dvp
	pstSnsInfo->port = 0; // sensor的逻辑编号，必须从0开始
	pstSnsInfo->dev_port = 0; // 每路 sensor 操作的驱动节点，一个驱动支持多个节点。 snsinfo 中的dev_port 必须等于pipeId，多目摄像头设置的时候需要特别注意
	pstSnsInfo->bus_type = 0; // 访问总线类型， 0 是 i2c,1 是 spi
	pstSnsInfo->bus_num = 2; // 总线号，根据具体板子硬件原理图确定 , 不配置默认 i2c5
	pstSnsInfo->fps = 10; // 帧率，用来选择使用哪一组帧率的sensor参数
	pstSnsInfo->resolution = RGB_HEIGHT; // sensor 行数, 必须要和mipi属性配置一致
	pstSnsInfo->sensor_addr = 0x37; // sensor i2c 设备地址
	pstSnsInfo->entry_index = 0; // sensor 使用的 mipi 索引, 0~3，对应mipi host的序号
	pstSnsInfo->sensor_mode = NORMAL_M; // sensor 工作模式， 1 是 normal,2 是dol2,3 是 dol3
	pstSnsInfo->reg_width = 8; // sensor 寄存器地址宽度
	pstSnsInfo->sensor_name = (char*)"gc2053"; // sensor的名字，在libcam.so中会根据这个名字组合出 libf37.so
											// 这个库文件，然后调用dlopen运行时打开sensor库， libf37.so 这个库的生成
											// 请参考sensor点亮文档，以f37 为例是 f37_utility.c f37_utility.h
											// 请放到sdk包的hbre/camera/utility/sensor目录下make生成libf37.so文件

	return pstSensorInfo;
}


MIPI_ATTR_S *GetGc2053MipiAttr(void)
{
	MIPI_ATTR_S *pstMipiAttr = &MIPI_2LANE_SENSOR_GC2053_10FPS_10BIT_LINEAR_ATTR;
	MIPI_HOST_CFG_S *pstHostCfg = &pstMipiAttr->mipi_host_cfg;
	
	memset(pstMipiAttr, 0, sizeof(*pstMipiAttr));

	pstHostCfg->lane = 1; // 硬件上sensor用了几个mipi数据lane，f37用的1 lane, os8a10 4K的用了4 lane
	pstHostCfg->datatype = 0x2b; // sensor的输出数据类型，请参考 《X3J3平台AIOT媒体系统接口手册.pdf》第3.5.36节 DATA TYPE
	pstHostCfg->mclk = 2400; // mipi 模块主时钟，目前默认是24MHz
	pstHostCfg->mipiclk = 864; // sensor 输出总的 mipibit rate, 单位Mbits/秒
	pstHostCfg->fps = 10; // sensor 输出实际帧率, 和 MIPI_SENSOR_INFO_S 是什么关系？
	pstHostCfg->width = RGB_WIDTH; // sensor实际输出的分辨率，一般根据sensor供应商的配置参数配置就行
	pstHostCfg->height = RGB_HEIGHT;
	pstHostCfg->linelenth = 2560; // sensor 输出带 blanking 的总行长， 一般根据sensor供应商的配置参数配置就行 2496
	pstHostCfg->framelenth = 1125; // sensor 输出带 blanking的总行数 13080
	pstHostCfg->settle = 20; // sensor 输出实际Ttx-zero + Ttx-prepare时间（ clk 为单位）, 一般0-100之间尝试 100
	pstHostCfg->channel_num = 1; // 使用虚通道的个数, linear 用一个，dol2 用 2 个， dol3 用3个
	//pstHostCfg->channel_sel = {0}; // 保存每个虚通道的值，在调用HB_VIN_SetDevVCNumber接口时用第一个，调用HB_VIN_AddDevVCNumber用后面的2 3 4的配置

	pstMipiAttr->dev_enable = 0; // mipi dev 是否使能， 1是使能， 0 是关闭, 只用开启mipi bypass的开启

	return pstMipiAttr;
}

