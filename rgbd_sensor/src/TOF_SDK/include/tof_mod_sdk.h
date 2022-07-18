#ifndef __TOF_MODULE_H__
#define __TOF_MODULE_H__

#include "typedef.h"
#include "tof_error.h"

#ifdef WIN32
    #ifdef TOF_MODULE_SDK_EXPORT
        #define TOFMDLL __declspec(dllexport)
    #else
        #define TOFMDLL __declspec(dllimport)
    #endif
#else
    #define TOFMDLL __attribute__((visibility("default")))
#endif


typedef struct tagTofModuleInitParam
{
	SCHAR szDepthCalcCfgFileDir[200];//深度计算所需配置文件的目录，如home/user/temp
	UINT8 nLogLevel;//日志打印级别

}TofModuleInitParam;

typedef struct tagTofModuleCaps
{	
	UINT32 supportedTOFMode;//TOF_MODE的组合
	UINT32 tofResWidth;
	UINT32 tofResHeight;
	GRAY_FORMAT grayFormat;//灰度数据格式

	//TOF HDRZ
	SBOOL bTofHDRZSupported;
	UINT8 byRes1[3];//字节对齐，预留

	//TOF RemoveINS
	SBOOL bTofRemoveINSSupported;
	//TOF MPIFlag
	SBOOL bTofMPIFlagSupported;//[该字段已作废]
	UINT8 byRes2[2];//字节对齐，预留

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER的组合

	//TOF Expouse
	UINT32 supportedTofExpMode;//EXP_MODE的组合[该字段已作废]

}TofModuleCaps;

typedef enum tagMODULE_NAME
{
	MODULE_NAME_FIRST = 0,

	MODULE_NAME_MD101D = MODULE_NAME_FIRST,
	MODULE_NAME_MTP004, 
	MODULE_NAME_MTP004C,
	MODULE_NAME_MTP006,
	MODULE_NAME_MTP007,
	MODULE_NAME_MTP008,
	MODULE_NAME_MTP009,
	MODULE_NAME_MTP009A,
	MODULE_NAME_MTP012,
	MODULE_NAME_MTT010,
	MODULE_NAME_MTT011,
	MODULE_NAME_MTT013,
	MODULE_NAME_MTT014,
	MODULE_NAME_MTT015,
	MODULE_NAME_MTT015A,
	MODULE_NAME_MTT016,
	MODULE_NAME_MTT020,
	MODULE_NAME_YMTT002,
	MODULE_NAME_YMTT003,
	MODULE_NAME_MTP013,



	MODULE_NAME_LAST,//新增模组型号放在该枚举值之前（为了兼容，需按先后顺序添加，不得改变原有枚举取值）
}MODULE_NAME;


//模组SDK的客户识别号
typedef enum tagMODULE_GUEST_ID
{
	MODULE_GUEST_ID_DEF = 0x00,//默认客户
	MODULE_GUEST_ID_01  = 0x01,//客户01
	MODULE_GUEST_ID_02  = 0x02,//客户02
	MODULE_GUEST_ID_03  = 0x03,//客户03
	MODULE_GUEST_ID_04  = 0x04,//客户04
	MODULE_GUEST_ID_05  = 0x05,//客户05
	MODULE_GUEST_ID_06  = 0x06,//客户06
	MODULE_GUEST_ID_07  = 0x07,//客户07
	MODULE_GUEST_ID_08  = 0x08,//客户08
	MODULE_GUEST_ID_09  = 0x09,//客户09

	MODULE_GUEST_ID_10  = 0x10,//客户10
	MODULE_GUEST_ID_11  = 0x11,//客户11
	MODULE_GUEST_ID_12  = 0x12,//客户12
	MODULE_GUEST_ID_13  = 0x13,//客户13
	MODULE_GUEST_ID_14  = 0x14,//客户14
	MODULE_GUEST_ID_15  = 0x15,//客户15
	MODULE_GUEST_ID_16  = 0x16,//客户16
	MODULE_GUEST_ID_17  = 0x17,//客户17
	MODULE_GUEST_ID_18  = 0x18,//客户18
	MODULE_GUEST_ID_19  = 0x19,//客户19

	MODULE_GUEST_ID_20  = 0x20,//客户20
	MODULE_GUEST_ID_21  = 0x21,//客户21
	MODULE_GUEST_ID_22  = 0x22,//客户22
	MODULE_GUEST_ID_23  = 0x23,//客户23
	MODULE_GUEST_ID_24  = 0x24,//客户24
	MODULE_GUEST_ID_25  = 0x25,//客户25
	MODULE_GUEST_ID_26  = 0x26,//客户26
	MODULE_GUEST_ID_27  = 0x27,//客户27
	MODULE_GUEST_ID_28  = 0x28,//客户28
	MODULE_GUEST_ID_29  = 0x29,//客户29


}MODULE_GUEST_ID;


typedef void* HTOFM;

typedef struct tagTofModuleHal
{
	SBOOL(*Init)(void* user_data);
	SBOOL(*Deinit)(void* user_data);
	
	/**********写入一个值到寄存器**********/
	//@    slave_addr:  从机地址;
	//@    regAddr:      寄存器地址;
	//@    value:           要写入寄存器的值;
	//@    返回值:       成功或失败;
	SBOOL(*WriteReg16)(const UINT8 slave_addr, const UINT16 regAddr, const UINT16 value, void*user_data);//根据模组实际情况选择是否实现

	/**********读取一个寄存器的值**********/
	//@    slave_addr:  从机地址;
	//@    regAddr:      寄存器地址;
	//@    value:           读取到的值;
	//@    返回值:       成功或失败;
	SBOOL(*ReadReg16)(const UINT8 slave_addr, const UINT16 regAddr, UINT16 *value, void*user_data);//根据模组实际情况选择是否实现

	/**********读取多个寄存器的值**********/
	//@    slave_addr:  从机地址;
	//@    startAddr:    要读取的起始地址;
	//@    addrCnt:      要读取的地址个数;
	//@    pBufOut:     存放读取到的数据;
	//@    返回值:      读取到的数据长度;
	UINT32(*ReadRegBulk)(const UINT8 slave_addr, const UINT32 startAddr, const UINT32 addrCnt, void *pBufOut, void*user_data);//根据模组实际情况选择是否实现


}TofModuleHal;


typedef struct tagTofModDepthData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

    //
	FLOAT32* pDepthData;//射线距离（滤波前）
	FLOAT32* pDepthDataFilter;//射线距离（滤波后）
	//
	PointData *pPointData;//点云数据
	//
	GRAY_FORMAT grayFormat;//pGrayData内数据格式
	void   *pGrayData;//灰度数据

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

	TofExpouseCurrentItems  autoExp;//计算出的自动曝光值（当需要自动曝光效果时，需要将该值设置到模组中）

}TofModDepthData;



typedef struct tagTofModDepthDataV20
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	//
	FLOAT32* pDepthData;//射线距离（滤波前）
	FLOAT32* pDepthDataFilter;//射线距离（滤波后）
	//
	PointData *pPointData;//点云数据
	//
	GRAY_FORMAT grayFormat;//pGrayData内数据格式
	void   *pGrayData;//灰度数据

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void   *pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

	TofExpouseCurrentItems  autoExp;//计算出的自动曝光值（当需要自动曝光效果时，需要将该值设置到模组中）


	/**************帧内HDRZ融合时特有数据************/

	//计算出的自动曝光帧数据（中间数据）
	FLOAT32* pDepthData_AEF;//射线距离
	PointData *pPointData_AEF;//点云数据
	GRAY_FORMAT grayFormat_AEF;//pGrayData_AEF内数据格式
	void   *pGrayData_AEF;//灰度数据

	//计算出的固定曝光帧数据（中间数据）
	FLOAT32* pDepthData_FEF;//射线距离
	PointData *pPointData_FEF;//点云数据
	GRAY_FORMAT grayFormat_FEF;//pGrayData_FEF内数据格式
	void   *pGrayData_FEF;//灰度数据


}TofModDepthDataV20;


typedef struct tagSomeCalibParam
{
	//
	TofModuleLensParameter struLensParamter;//TOF模组内参和畸变（V1.0版本）建议不要再用，因为不能适用于鱼眼模型）
	TofModuleLensParameterV20 struLensParamterV20;//TOF模组内参和畸变（V2.0版本）

	//

}SomeCalibParam;



#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/*********************第一部分：SDK基本接口（公用）**************************/
/*****************************************************************************/

//初始化/反初始化SDK（其他任何接口的使用都必须介于这两个接口之间）
TOFMDLL TOFRET TOFM_Init(TofModuleInitParam* pInitParam);
TOFMDLL TOFRET TOFM_Uninit(void);

//获取SDK版本号（返回值为字符串型版本号）
TOFMDLL SCHAR* TOFM_GetSDKVersion(void);

//创建/释放句柄资源
//老接口（逐渐废弃，仅适用于MODULE_NAME枚举）
TOFMDLL HTOFM  TOFM_OpenDevice(const MODULE_NAME mod_name, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//新接口（兼容老接口，但是MODULE_NAME枚举不支持的模组型号一定要用新接口）
TOFMDLL HTOFM  TOFM_OpenDeviceV20(SCHAR* pModName, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//新接口（兼容老接口，但是MODULE_NAME枚举不支持的模组型号一定要用新接口，相对于V20接口，可以实现不同客户对同一模组的不同数据需求）
TOFMDLL HTOFM  TOFM_OpenDeviceV30(SCHAR* pModName, const MODULE_GUEST_ID guestID, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
TOFMDLL TOFRET TOFM_CloseDevice(HTOFM hTofMod);

//绑定TOF 模式（关系到后续的标定数据解析、深度计算初始化和取流功能）
//老接口（采用默认的配置文件）
TOFMDLL TOFRET TOFM_SetTofMode(HTOFM hTofMod, const TOF_MODE tofMode);
//新接口（可以重新指定配置文件）, pModCfgFile为配置文件完整路径并且可以为NULL值（NULL时表示采用默认配置文件）
TOFMDLL TOFRET TOFM_SetTofModeV20(HTOFM hTofMod, const TOF_MODE tofMode, SCHAR* pModCfgFile);

//绑定一些回调函数（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_SetExterntionHooks(HTOFM hTofMod, ExterntionHooks* pHooks);


/*****************************************************************************/
/*********************第二部分：该部分为模组硬件相关操作（硬件交互）**********/
/*****************************************************************************/

//从模组里读取标定数据
TOFMDLL UINT32 TOFM_ReadCalibData(HTOFM hTofMod, UINT8* pCalibData, const UINT32 nBufLen);
//获取/设置模组曝光
TOFMDLL TOFRET TOFM_GetTofExpTime(HTOFM hTofMod, TofExpouseItems* pExp);
TOFMDLL TOFRET TOFM_SetTofExpTime(HTOFM hTofMod, TofExpouseCurrentItems* pExp);
//获取模组温度
TOFMDLL TOFRET TOFM_GetTemperature(HTOFM hTofMod, FLOAT32* pTemperature);
//开启/关闭模组MIPI出流
TOFMDLL TOFRET TOFM_StartTofStream(HTOFM hTofMod);
TOFMDLL TOFRET TOFM_StopTofStream(HTOFM hTofMod);


/*****************************************************************************/
/*********************第三部分：该部分为深度计算相关算法（软件算法）**********/
/*****************************************************************************/

//解析/释放标定数据
TOFMDLL TOFRET TOFM_LoadCalibData(HTOFM hTofMod, UINT8* pCalibData, const UINT32 nCalibDataLen);
TOFMDLL TOFRET TOFM_UnLoadCalibData(HTOFM hTofMod);
//读取标定数据内部分标定参数
TOFMDLL TOFRET TOFM_GetSomeCalibParam(HTOFM hTofMod, SomeCalibParam* pParamOut); //（该接口必须在TOFM_LoadCalibData之后，TOFM_UnLoadCalibData之前调用）

//深度计算模块（该部分必须在TOFM_LoadCalibData之后，TOFM_UnLoadCalibData之前调用）
TOFMDLL TOFRET TOFM_InitDepthCal(HTOFM hTofMod);//(比较耗时)
TOFMDLL TOFRET TOFM_UnInitDepthCal(HTOFM hTofMod);
TOFMDLL TOFRET TOFM_DoDepthCal(HTOFM hTofMod, TofRawData* pRawData, TofModDepthData* pDataOut);//（该接口必须在TOFM_InitDepthCal之后，TOFM_UnInitDepthCal之前调用）
TOFMDLL TOFRET TOFM_DoDepthCalV20(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//（该接口必须在TOFM_InitDepthCal之后，TOFM_UnInitDepthCal之前调用）

TOFMDLL TOFRET TOFM_DoDepthCalV20_OnlyAEExp(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//（该接口必须在TOFM_InitDepthCal之后，TOFM_UnInitDepthCal之前调用）
TOFMDLL TOFRET TOFM_DoDepthCalV20_OnlyDepth(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//（该接口必须在TOFM_InitDepthCal之后，TOFM_UnInitDepthCal之前调用）


//启用/禁用AE算法[已弃用]
TOFMDLL TOFRET TOFM_SetTofAE(HTOFM hTofMod, const SBOOL bEnable);
//启用/禁用某种滤波算法
TOFMDLL TOFRET TOFM_GetTofFilter(HTOFM hTofMod, const TOF_FILTER type, SBOOL* pbEnable);
TOFMDLL TOFRET TOFM_SetTofFilter(HTOFM hTofMod, const TOF_FILTER type, const SBOOL bEnable);
//启用/禁用某种滤波算法（带具体参数）
TOFMDLL TOFRET TOFM_GetTofFilterV20(HTOFM hTofMod, TofFilterCfg* pCfg);
TOFMDLL TOFRET TOFM_SetTofFilterV20(HTOFM hTofMod, TofFilterCfg* pCfg);

//启用/禁用HDRZ算法
TOFMDLL TOFRET TOFM_SetTofHDRZ(HTOFM hTofMod, const SBOOL bEnable);

//启用/禁用RemoveINS算法
TOFMDLL TOFRET TOFM_SetTofRemoveINS(HTOFM hTofMod, const SBOOL bEnable);

//启用/禁用MPIFlag算法[已弃用，请使用TOFM_SetTofFilter(xxx, TOF_FILTER_MPIFilter, xxx)]
TOFMDLL TOFRET TOFM_SetTofMPIFlag(HTOFM hTofMod, const SBOOL bEnable);

//调整raw数据的格式（非必须函数，按需调用，常见于嵌入式平台下大小端转换）
TOFMDLL TOFRET TOFM_ConvertRawData(HTOFM hTofMod, UINT8* pRaw, const UINT32 nRawLen, UINT8* pRawOut, const UINT32 nOutBufLen);

//调整深度计算的ROI（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);
TOFMDLL TOFRET TOFM_SetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);

//获取/调整曝光上下限（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetTofExpTimeRange(HTOFM hTofMod, TofExpouseRangeItems* pRange);
TOFMDLL TOFRET TOFM_SetTofExpTimeRange(HTOFM hTofMod, TofExpouseRangeItems* pRange);

//获取/调整客户们自定义要求的数据（非必须函数，按需调用）
TOFMDLL TOFRET TOFM_GetGuestCustomParam(HTOFM hTofMod, GuestCustomParam* pParam);
TOFMDLL TOFRET TOFM_SetGuestCustomParam(HTOFM hTofMod, GuestCustomParam* pParam);



#ifdef __cplusplus
}
#endif

#endif

