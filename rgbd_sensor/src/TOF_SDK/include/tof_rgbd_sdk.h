#ifndef __TOF_RGBD_SDK_H__
#define __TOF_RGBD_SDK_H__


#ifdef WIN32
    #ifdef TOF_RGBD_SDK_EXPORT
        #define TOFRGBDDLL __declspec(dllexport)
    #else
        #define TOFRGBDDLL __declspec(dllimport)
    #endif
#else
    #define TOFRGBDDLL __attribute__((visibility("default")))
#endif

#ifndef NULL
	#define NULL 0
#endif


//错误码
typedef enum tagTOFRGBDRET
{
	TOFRGBDRET_SUCCESS              = 0x00000000,//成功
		
	TOFRGBDRET_FAILED               = 0x80000001,//失败
	TOFRGBDRET_ERROR_INVALID_PARAM  = 0x80000002,//无效参数
	TOFRGBDRET_ERROR_ACCESS         = 0x80000003,//无权限
	TOFRGBDRET_ERROR_OVERFLOW       = 0x80000004,//溢出
	TOFRGBDRET_ERROR_NO_MEM         = 0x80000005,//内存不足
	TOFRGBDRET_ERROR_WRONG_STATUS   = 0x80000006,//状态错误
	TOFRGBDRET_ERROR_NOT_SUPPORTED  = 0x80000007,//功能不支持
	TOFRGBDRET_ERROR_DATA           = 0x80000008,//错误的数据
	


	TOFRGBDRET_ERROR_OTHER          = 0x8fffffff,//其他未特定指明的错误
}TOFRGBDRET;

//RGBD SDK的客户识别号
typedef enum tagTOFRGBD_GUEST_ID
{
	TOFRGBD_GUEST_ID_DEF = 0x00,//默认客户
	TOFRGBD_GUEST_ID_01 = 0x01,//客户01
	TOFRGBD_GUEST_ID_02 = 0x02,//客户02
	TOFRGBD_GUEST_ID_03 = 0x03,//客户03
	TOFRGBD_GUEST_ID_04 = 0x04,//客户04
	TOFRGBD_GUEST_ID_05 = 0x05,//客户05
	TOFRGBD_GUEST_ID_06 = 0x06,//客户06
	TOFRGBD_GUEST_ID_07 = 0x07,//客户07
	TOFRGBD_GUEST_ID_08 = 0x08,//客户08
	TOFRGBD_GUEST_ID_09 = 0x09,//客户09

	TOFRGBD_GUEST_ID_10 = 0x10,//客户10
	TOFRGBD_GUEST_ID_11 = 0x11,//客户11
	TOFRGBD_GUEST_ID_12 = 0x12,//客户12
	TOFRGBD_GUEST_ID_13 = 0x13,//客户13
	TOFRGBD_GUEST_ID_14 = 0x14,//客户14
	TOFRGBD_GUEST_ID_15 = 0x15,//客户15
	TOFRGBD_GUEST_ID_16 = 0x16,//客户16
	TOFRGBD_GUEST_ID_17 = 0x17,//客户17
	TOFRGBD_GUEST_ID_18 = 0x18,//客户18
	TOFRGBD_GUEST_ID_19 = 0x19,//客户19

	TOFRGBD_GUEST_ID_20 = 0x20,//客户20
	TOFRGBD_GUEST_ID_21 = 0x21,//客户21
	TOFRGBD_GUEST_ID_22 = 0x22,//客户22
	TOFRGBD_GUEST_ID_23 = 0x23,//客户23
	TOFRGBD_GUEST_ID_24 = 0x24,//客户24
	TOFRGBD_GUEST_ID_25 = 0x25,//客户25
	TOFRGBD_GUEST_ID_26 = 0x26,//客户26
	TOFRGBD_GUEST_ID_27 = 0x27,//客户27
	TOFRGBD_GUEST_ID_28 = 0x28,//客户28
	TOFRGBD_GUEST_ID_29 = 0x29,//客户29


}TOFRGBD_GUEST_ID;


//灰度格式
typedef enum tagTofRgbd_Gray_Format
{
	TofRgbd_Gray_Format_U8    = 0,//U8格式
	TofRgbd_Gray_Format_U16   = 1,//U16格式
	TofRgbd_Gray_Format_Float = 2,//float格式
}TofRgbd_Gray_Format;

//RGB或TOF模组内参和畸变 (通用模型)
typedef struct tagTofRgbdLensGeneral
{
	float fx;
	float fy;
	float cx;
	float cy;
	float k1;
	float k2;
	float p1;
	float p2;
	float k3;
}TofRgbdLensGeneral;

//RGB或TOF模组内参和畸变 (鱼眼模型)
typedef struct tagTofRgbdLensFishEye
{
	float fx;
	float fy;
	float cx;
	float cy;
	float k1;
	float k2;
	float k3;
	float k4;
}TofRgbdLensFishEye;

//RGB或TOF模组内参和畸变
typedef struct tagTofRgbdLensParameter
{
	int nIndex;//1---general有效, 2---fishEye有效

	union
	{
		//[第1种]: 普通模型
		TofRgbdLensGeneral general;//普通模型

		//[第2种]: 鱼眼模型
		TofRgbdLensFishEye fishEye;//鱼眼模型
	}uParam;

}TofRgbdLensParameter;

//RGBD参数类型
typedef enum tagTOF_RGBD_PARAM_TYPE
{
	TOF_RGBD_PARAM_TofCameraLensParam = 0, //TOF模组的内参
	TOF_RGBD_PARAM_RgbCameraLensParam, //RGB模组的内参
}TOF_RGBD_PARAM_TYPE;

//RGBD参数
typedef struct tagTofRgbdParameter
{
	TOF_RGBD_PARAM_TYPE type;//输入参数，只读

	union
	{
		TofRgbdLensParameter lensParam; //RGB或TOF模组的内参, 当 type=TOF_RGBD_PARAM_TofCameraLensParam 或 TOF_RGBD_PARAM_RgbCameraLensParam 时有效

	}uParam;

}TofRgbdParameter;


//坐标
typedef struct tagTofRgbdPixelCoord
{
	unsigned short x;
	unsigned short y;
}TofRgbdPixelCoord;

//点云
typedef struct tagTofRgbdPointCloud
{
	float x;
	float y;
	float z;
}TofRgbdPointCloud;

//彩色点云
typedef struct tagTofRgbdPointCloudColor
{
	float x;
	float y;
	float z;

	float b;
	float g;
	float r;
}TofRgbdPointCloudColor;


//客户定制的私有数据
typedef struct tagTofRgbdImage_Priv
{
	void*        pData;
	unsigned int nDataLen; //pData内数据字节数
}TofRgbdImage_Priv;

//未指定格式的图像信息
typedef struct tagTofRgbdImage_Void
{
	void*        pData;
	unsigned int nWidth;
	unsigned int nHeight;
	unsigned int nDataLen; //pData内数据字节数
}TofRgbdImage_Void;

//U8图像信息
typedef struct tagTofRgbdImage_U8
{
	unsigned char* pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int   nWidth;
	unsigned int   nHeight;
}TofRgbdImage_U8;

//U16图像信息
typedef struct tagTofRgbdImage_U16
{
	unsigned short* pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int    nWidth;
	unsigned int    nHeight;
}TofRgbdImage_U16;

//float图像信息
typedef struct tagTofRgbdImage_Float
{
	float*       pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int nWidth;
	unsigned int nHeight;
}TofRgbdImage_Float;

//点云图像信息
typedef struct tagTofRgbdImage_PointCloud
{
	TofRgbdPointCloud* pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int       nWidth;
	unsigned int       nHeight;
}TofRgbdImage_PointCloud;

//彩色点云图像信息
typedef struct tagTofRgbdImage_PointCloudColor
{
	TofRgbdPointCloudColor* pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int            nWidth;
	unsigned int            nHeight;
}TofRgbdImage_PointCloudColor;

//坐标信息
typedef struct tagTofRgbdImage_PixelCoord
{
	TofRgbdPixelCoord* pData;//数据总长：nWidth * nHeight * sizeof(pData[0])
	unsigned int    nWidth;
	unsigned int    nHeight;
}TofRgbdImage_PixelCoord;


//创建RGBD计算句柄的初始化参数
typedef struct tagTofRgbdHandleParam
{
	char szModuleName[32];//模组型号
	TOFRGBD_GUEST_ID guestID;//客户识别号

	unsigned char* pRgbdCalibData;//RGBD标定数据
	unsigned int   nRgbdCalibDataLen;//pRgbdCalibData内RGBD标定数据长度（字节数）

	unsigned int nTofWidth;//TOF分辨率宽
	unsigned int nTofHeight;//TOF分辨率高

	unsigned int nRgbWidth;//RGB分辨率宽
	unsigned int nRgbHeight;//RGB分辨率高

	TofRgbd_Gray_Format inGrayFormat;//输入的灰度格式

}TofRgbdHandleParam;


//RGBD计算的输入参数
typedef struct tagTofRgbdInputData
{
	TofRgbdPointCloud*  pPointCloud;//点云数据（以米为单位）

	void*        pGray;//灰度数据（格式必须与初始化参数inGrayFormat指定的一致）
	unsigned int nGrayLen;//pGray内灰度数据长度（字节数）

	unsigned char* pRgb;//BGR数据，只能是 bgr 排列顺序
	unsigned int   nRgbLen;//pRgb内RGB数据长度（字节数）

}TofRgbdInputData;

//RGBD计算的输出参数
typedef struct tagTofRgbdOutputData
{
	TofRgbdImage_PointCloud pointCloud2Rgb;//配准后的点云（以米为单位）（可能为空）
	TofRgbdImage_U8   gray2Rgb;//配准后的灰度数据（可能为空）
	TofRgbdImage_Void rgb2Tof;//配准后的RGB数据（格式与输入的相同）（可能为空）
	TofRgbdImage_PointCloudColor colorPointCloud;//配准后的彩色点云数据（可能为空）

	 
	TofRgbdImage_PixelCoord rgb2TofPixelCoord; //RGB坐标与TOF坐标的映射表（可能为空）
	
	TofRgbdImage_Priv privData;//私有数据（用于客户定制）（一般为空）

}TofRgbdOutputData;


//RGBD计算的句柄
typedef void* HTOFRGBD;


#ifdef __cplusplus
extern "C" {
#endif


//获取SDK版本号（返回值为字符串型版本号）
TOFRGBDDLL char* TOFRGBD_GetSDKVersion(void);

//创建/释放句柄资源
TOFRGBDDLL HTOFRGBD   TOFRGBD_CreateHandle(TofRgbdHandleParam* pInputParam);
TOFRGBDDLL TOFRGBDRET TOFRGBD_CloseHandle(HTOFRGBD hTofRgbd);
//RGBD计算
TOFRGBDDLL TOFRGBDRET TOFRGBD_DoCal(HTOFRGBD hTofRgbd, TofRgbdInputData* pDataIn, TofRgbdOutputData* pDataOut);//（该接口必须在TOFRGBD_CreateHandle之后，TOFRGBD_CloseHandle之前调用）


//获取/设置模组参数
TOFRGBDDLL TOFRGBDRET TOFRGBD_GetParameters(HTOFRGBD hTofRgbd, TofRgbdParameter* pParam);
TOFRGBDDLL TOFRGBDRET TOFRGBD_SetParameters(HTOFRGBD hTofRgbd, TofRgbdParameter* pParam);


#ifdef __cplusplus
}
#endif

#endif

