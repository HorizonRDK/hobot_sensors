#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__


typedef unsigned int       UINT32;
typedef unsigned short     UINT16;
typedef unsigned char	   UINT8;
typedef unsigned long long UINT64;

typedef signed int    	   SINT32;
typedef signed short  	   SINT16;
typedef signed char		   SINT8;
typedef signed long long   SINT64;

typedef float              FLOAT32;
typedef double 		       FLOAT64;
typedef bool			   SBOOL;
typedef char			   SCHAR;	


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef NULL
    #define NULL 0
#endif


#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))


typedef enum tagTOF_MODE
{
	//双频
	TOF_MODE_STERO_5FPS  = 0x00000001,
	TOF_MODE_STERO_10FPS = 0x00000002,
	TOF_MODE_STERO_15FPS = 0x00000004,
	TOF_MODE_STERO_30FPS = 0x00000008,
	TOF_MODE_STERO_45FPS = 0x00000010,
	TOF_MODE_STERO_60FPS = 0x00000020,

	//单频
	TOF_MODE_MONO_5FPS   = 0x00000040,
	TOF_MODE_MONO_10FPS  = 0x00000080,
	TOF_MODE_MONO_15FPS  = 0x00000100,
	TOF_MODE_MONO_30FPS  = 0x00000200,
	TOF_MODE_MONO_45FPS  = 0x00000400,
	TOF_MODE_MONO_60FPS  = 0x00000800,

	//HDRZ：这几个模式代表具有raw数据的HDRZ融合的
	TOF_MODE_HDRZ_5FPS   = 0x00001000,
	TOF_MODE_HDRZ_10FPS  = 0x00002000,
	TOF_MODE_HDRZ_15FPS  = 0x00004000,
	TOF_MODE_HDRZ_30FPS  = 0x00008000,
	TOF_MODE_HDRZ_45FPS  = 0x00010000,
	TOF_MODE_HDRZ_60FPS  = 0x00020000,

	//帧率不同
	TOF_MODE_5FPS        = 0x00040000,
	TOF_MODE_10FPS       = 0x00080000,
	TOF_MODE_20FPS       = 0x00100000,
	TOF_MODE_30FPS       = 0x00200000,
	TOF_MODE_45FPS       = 0x00400000,
	TOF_MODE_60FPS       = 0x00800000,

	//ADI特定
	TOF_MODE_ADI_1M5     = 0x01000000,
	TOF_MODE_ADI_5M      = 0x02000000,

	//自定义
	TOF_MODE_CUSTOM_1    = 0x04000000,
	TOF_MODE_CUSTOM_2    = 0x08000000,
	TOF_MODE_CUSTOM_3    = 0x10000000,
	TOF_MODE_CUSTOM_4    = 0x20000000,
	TOF_MODE_CUSTOM_5    = 0x40000000,

	//DEBUG模式
	TOF_MODE_DEBUG       = 0x80000000,


}TOF_MODE;


typedef enum tagTOF_FILTER
{
	TOF_FILTER_RemoveFlyingPixel   = 0x00000001,
	TOF_FILTER_AdaptiveNoiseFilter = 0x00000002,
	TOF_FILTER_InterFrameFilter    = 0x00000004,
	TOF_FILTER_PointCloudFilter    = 0x00000008,
	TOF_FILTER_StraylightFilter    = 0x00000010,
	TOF_FILTER_CalcIntensities     = 0x00000020,
	TOF_FILTER_MPIFlagAverage      = 0x00000040,
	TOF_FILTER_MPIFlagAmplitude    = 0x00000080,
	TOF_FILTER_MPIFlagDistance     = 0x00000100,
	TOF_FILTER_ValidateImage       = 0x00000200,
	TOF_FILTER_SparsePointCloud    = 0x00000400,
	TOF_FILTER_Average             = 0x00000800,
	TOF_FILTER_Median              = 0x00001000,
	TOF_FILTER_Confidence          = 0x00002000,
	TOF_FILTER_MPIFilter           = 0x00004000,
	TOF_FILTER_PointCloudCorrect   = 0x00008000,
	TOF_FILTER_LineRecognition     = 0x00010000,
	TOF_FILTER_RadialFusion        = 0x00020000,

}TOF_FILTER;


typedef struct tagTofFilterCfg_RemoveFlyingPixel
{
	FLOAT32 f0;
	FLOAT32 f1;
	FLOAT32 nd;
	FLOAT32 fd;
}TofFilterCfg_RemoveFlyingPixel;

typedef struct tagTofFilterCfg_AdaptiveNoiseFilter
{
	SINT32  k;
	FLOAT32 s;
	SINT32  t;
}TofFilterCfg_AdaptiveNoiseFilter;

typedef struct tagTofFilterCfg_InterFrameFilter
{
	FLOAT32 mdg;
	FLOAT32 mdt;
	FLOAT32 fg1;
	FLOAT32 fg2;
}TofFilterCfg_InterFrameFilter;

typedef struct tagTofFilterCfg_PointCloudFilter
{
	SINT32 k;
}TofFilterCfg_PointCloudFilter;

typedef struct tagTofFilterCfg_StraylightFilter
{
	FLOAT32 d[16];
	FLOAT32 t[16];
}TofFilterCfg_StraylightFilter;

typedef struct tagTofFilterCfg_CalcIntensities
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_CalcIntensities;

typedef struct tagTofFilterCfg_MPIFlagAverage
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_MPIFlagAverage;

typedef struct tagTofFilterCfg_MPIFlagAmplitude
{
	FLOAT32 mat;
	FLOAT32 ndt;
}TofFilterCfg_MPIFlagAmplitude;

typedef struct tagTofFilterCfg_MPIFlagDistance
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_MPIFlagDistance;

typedef struct tagTofFilterCfg_ValidateImage
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_ValidateImage;

typedef struct tagTofFilterCfg_SparsePointCloud
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_SparsePointCloud;

typedef struct tagTofFilterCfg_Average
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_Average;

typedef struct tagTofFilterCfg_Median
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_Median;

typedef struct tagTofFilterCfg_Confidence
{
	FLOAT32 t;
}TofFilterCfg_Confidence;

typedef struct tagTofFilterCfg_MPIFilter
{
	FLOAT32 ndt;
	FLOAT32 fdt;
	FLOAT32 nnr;
	FLOAT32 mnr;
	FLOAT32 fnr;
	FLOAT32 rd;
}TofFilterCfg_MPIFilter;

typedef struct tagTofFilterCfg_PointCloudCorrect
{
	FLOAT32 da;//模组下斜的角度
	FLOAT32 tgd;//模组距地面的高度
	FLOAT32 t1;
	FLOAT32 t2;
}TofFilterCfg_PointCloudCorrect;

typedef struct tagTofFilterCfg_LineRecognition
{
	SINT32 ht;
	SINT32 cgt;
	SINT32 fgst;
	FLOAT32 gstr;
	SINT32 spgt;
	SINT32 opgt;
	SINT32 rc;
	SINT32 lc;
	SINT32 ma;
}TofFilterCfg_LineRecognition;

typedef struct tagTofFilterCfg_RadialFusion
{
	UINT8 szRes[4];//预留,4字节对齐
}TofFilterCfg_RadialFusion;


typedef struct tagTofFilterCfg
{
	TOF_FILTER type;//某一种滤波类型，一般是输入参数（只读）

	union
	{
		SBOOL bEnable;//是否启用，可以是输入或者输出参数(暂时不开放,目前属于无效字段)
		UINT8 szRes[4];//预留,4字节对齐
	}uRes;

	union
	{
		TofFilterCfg_RemoveFlyingPixel struRemoveFlyingPixel;//当type取值为TOF_FILTER_RemoveFlyingPixel时有效
		TofFilterCfg_AdaptiveNoiseFilter struAdaptiveNoiseFilter;//当type取值为TOF_FILTER_AdaptiveNoiseFilter时有效
		TofFilterCfg_InterFrameFilter struInterFrameFilter;//当type取值为TOF_FILTER_InterFrameFilter时有效
		TofFilterCfg_PointCloudFilter struPointCloudFilter;//当type取值为TOF_FILTER_PointCloudFilter时有效
		TofFilterCfg_StraylightFilter struStraylightFilter;//当type取值为TOF_FILTER_StraylightFilter时有效
		TofFilterCfg_CalcIntensities struCalcIntensities;//当type取值为TOF_FILTER_CalcIntensities时有效
		TofFilterCfg_MPIFlagAverage struMPIFlagAverage;//当type取值为TOF_FILTER_MPIFlagAverage时有效
		TofFilterCfg_MPIFlagAmplitude struMPIFlagAmplitude;//当type取值为TOF_FILTER_MPIFlagAmplitude时有效
		TofFilterCfg_MPIFlagDistance struMPIFlagDistance;//当type取值为TOF_FILTER_MPIFlagDistance时有效
		TofFilterCfg_ValidateImage struValidateImage;//当type取值为TOF_FILTER_ValidateImage时有效
		TofFilterCfg_SparsePointCloud struSparsePointCloud;//当type取值为TOF_FILTER_SparsePointCloud时有效
		TofFilterCfg_Average struAverage;//当type取值为TOF_FILTER_Average时有效
		TofFilterCfg_Median struMedian;//当type取值为TOF_FILTER_Median时有效
		TofFilterCfg_Confidence struConfidence;//当type取值为TOF_FILTER_Confidence时有效
		TofFilterCfg_MPIFilter struMPIFilter;//当type取值为TOF_FILTER_MPIFilter时有效
		TofFilterCfg_PointCloudCorrect struPointCloudCorrect;//当type取值为TOF_FILTER_PointCloudCorrect时有效
		TofFilterCfg_LineRecognition struLineRecognition;//当type取值为TOF_FILTER_LineRecognition时有效
		TofFilterCfg_RadialFusion struRadialFusion;//当type取值为TOF_FILTER_RadialFusion时有效
	}uCfg;//某种滤波的具体配置，可以是输入或者输出参数

}TofFilterCfg;

typedef enum tagEXP_MODE
{
	EXP_MODE_MANUAL = 0x00000001,//手动曝光
	EXP_MODE_AUTO   = 0x00000002,//自动曝光(AE)
}EXP_MODE;



//灰度数据格式
typedef enum tagGRAY_FORMAT
{
	GRAY_FORMAT_UINT8  = 0,//8位数据
	GRAY_FORMAT_UINT16,//无符号16位数据
	GRAY_FORMAT_FLOAT,//浮点型数据
	GRAY_FORMAT_BGRD,//每像素32位， 按B/G/R/D顺序存放

}GRAY_FORMAT;


typedef struct tagPointData
{
	FLOAT32 x;
	FLOAT32 y;
	FLOAT32 z;
}PointData;

typedef struct tagRgbDData
{
	UINT8 b;
	UINT8 g;
	UINT8 r;
}RgbDData;

//坐标
typedef struct tagPixelCoordData
{
	UINT16 x;
	UINT16 y;
}PixelCoordData;

typedef enum tagCOLOR_FORMAT
{
	//MJPG格式
	COLOR_FORMAT_MJPG     = MAKE_UNIQUE_ID('M', 'J', 'P', 'G'),

	//H264格式
	COLOR_FORMAT_H264     = MAKE_UNIQUE_ID('H', '2', '6', '4'),

	//YUV格式
	COLOR_FORMAT_YUV422   = MAKE_UNIQUE_ID('Y', 'U', 'V', 0x22),
	COLOR_FORMAT_YUYV     = MAKE_UNIQUE_ID('Y', 'U', 'Y', 'V'),
	COLOR_FORMAT_I420     = MAKE_UNIQUE_ID('I', '4', '2', '0'),
	COLOR_FORMAT_YV12     = MAKE_UNIQUE_ID('Y', 'V', '1', '2'),
	COLOR_FORMAT_NV12     = MAKE_UNIQUE_ID('N', 'V', '1', '2'),
	COLOR_FORMAT_NV21     = MAKE_UNIQUE_ID('N', 'V', '2', '1'),

	//RGB格式
	COLOR_FORMAT_BGR      = MAKE_UNIQUE_ID('B', 'G', 'R', 0x00), //RGB24（每个像素占3个字节，按照B、G、R的顺序存放）
	COLOR_FORMAT_RGB      = MAKE_UNIQUE_ID('R', 'G', 'B', 0x00), //RGB24（每个像素占3个字节，按照R、G、B的顺序存放）
	COLOR_FORMAT_BGRA     = MAKE_UNIQUE_ID('B', 'G', 'R', 'A'), //RGB32（每个像素占4个字节，按照B、G、R、A的顺序存放）
	COLOR_FORMAT_RGBA     = MAKE_UNIQUE_ID('R', 'G', 'B', 'A'), //RGB32（每个像素占4个字节，按照R、G、B、A的顺序存放）

}COLOR_FORMAT;


typedef struct tagRgbData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbData;


//RGB模组内参和畸变
typedef struct tagRgbModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}RgbModuleLensParameter;

//双目相机参数
typedef struct tagStereoLensParameter
{
	FLOAT32 szRotationMatrix[3][3];//双目旋转矩阵
	FLOAT32 szTranslationMatrix[3];//双目平移矩阵

}StereoLensParameter;


//TOF Expouse
typedef struct tagTofExpouse
{
	UINT32  	nCurrent;//当前值，可读写
	UINT32  	nDefault;//默认值，只读
	UINT32  	nStep;//步进值，只读
	UINT32  	nMax;//最大值，只读
	UINT32  	nMin;//最小值，只读
}TofExpouse;


//TOF Expouse Group1
typedef struct tagTofExpouseGroup1
{
	TofExpouse exp;//曝光参数
}TofExpouseGroup1;


//TOF Expouse Group2
typedef struct tagTofExpouseGroup2
{
	TofExpouse exp_AEF;//自动曝光帧曝光参数
	TofExpouse exp_FEF;//固定曝光帧曝光参数
}TofExpouseGroup2;


//TOF Expouse Group3
typedef struct tagTofExpouseGroup3
{
	TofExpouse exp_AEF;//自动曝光帧曝光参数
	TofExpouse exp_FEF;//固定曝光帧曝光参数
	TofExpouse exp_Gray;//灰度曝光帧曝光参数
}TofExpouseGroup3;

//TOF Expouse Items
typedef struct tagTofExpouseItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效, 3---g3有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseGroup1 g1;//曝光参数

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseGroup2 g2;//曝光参数

		//[第3种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时），并且还可以配置灰度曝光帧的曝光
		TofExpouseGroup3 g3;//曝光参数
	}uParam;

}TofExpouseItems;

//TOF Expouse Current Group1
typedef struct tagTofExpouseCurrentGroup1
{
	UINT32 exp;//曝光值
}TofExpouseCurrentGroup1;


//TOF Expouse Current Group2
typedef struct tagTofExpouseCurrentGroup2
{
	UINT32 exp_AEF;//自动曝光帧曝光值
	UINT32 exp_FEF;//固定曝光帧曝光值
}TofExpouseCurrentGroup2;


//TOF Expouse Current Group3
typedef struct tagTofExpouseCurrentGroup3
{
	UINT32 exp_AEF;//自动曝光帧曝光值
	UINT32 exp_FEF;//固定曝光帧曝光值
	UINT32 exp_Gray;//灰度曝光帧曝光值
}TofExpouseCurrentGroup3;

//TOF Expouse Current Items
typedef struct tagTofExpouseCurrentItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效, 3---g3有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseCurrentGroup1 g1;//曝光值

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseCurrentGroup2 g2;//曝光值

		//[第3种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时），并且还可以配置灰度曝光帧的曝光
		TofExpouseCurrentGroup3 g3;//曝光值
	}uParam;

}TofExpouseCurrentItems;


//TOF Expouse Range Group1
typedef struct tagTofExpouseRangeGroup1
{
	UINT32 min;//曝光值(最小)
	UINT32 max;//曝光值(最大)
}TofExpouseRangeGroup1;


//TOF Expouse Range Group2
typedef struct tagTofExpouseRangeGroup2
{
	UINT32 min_AEF;//自动曝光帧曝光值(最小)
	UINT32 max_AEF;//自动曝光帧曝光值(最大)
	
	UINT32 min_FEF;//固定曝光帧曝光值(最小)
	UINT32 max_FEF;//固定曝光帧曝光值(最大)
}TofExpouseRangeGroup2;


//TOF Expouse Range Group3
typedef struct tagTofExpouseRangeGroup3
{
	UINT32 min_AEF;//自动曝光帧曝光值(最小)
	UINT32 max_AEF;//自动曝光帧曝光值(最大)

	UINT32 min_FEF;//固定曝光帧曝光值(最小)
	UINT32 max_FEF;//固定曝光帧曝光值(最大)

	UINT32 min_Gray;//灰度曝光帧曝光值(最小)
	UINT32 max_Gray;//灰度曝光帧曝光值(最大)
}TofExpouseRangeGroup3;


//TOF Expouse Range Items
typedef struct tagTofExpouseRangeItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效, 3---g3有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseRangeGroup1 g1;//曝光范围

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseRangeGroup2 g2;//曝光范围

		//[第3种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时），并且还可以配置灰度曝光帧的曝光
		TofExpouseRangeGroup3 g3;//曝光范围
	}uParam;

}TofExpouseRangeItems;


//自定义参数客户识别号
typedef enum tagCUSTOM_PARAM_GUEST_ID
{
	CUSTOM_PARAM_GUEST_ID_1 = 1,//客户1
	CUSTOM_PARAM_GUEST_ID_2 = 2,//客户2

}CUSTOM_PARAM_GUEST_ID;

//客户1自定义的参数
typedef struct tagCustomParamGuest1
{
	SINT32 quantileThreshold;//AE 比例
	FLOAT32 referenceAmplitude;//参考幅度
	FLOAT32 amplitudeThreshold;//幅度阈值
	UINT8 szRes[496];//总长508字节，4字节对齐，预留
}CustomParamGuest1;

//客户2自定义的参数
typedef struct tagCustomParamGuest2
{
	UINT8 szRes[508];//总长508字节，4字节对齐，预留
}CustomParamGuest2;


//客户自定义的参数
typedef struct tagGuestCustomParam
{
	CUSTOM_PARAM_GUEST_ID id;//输入参数，只读

	union
	{
		CustomParamGuest1 p1;//当id为CUSTOM_PARAM_GUEST_ID_1时有效；
		CustomParamGuest2 p2;//当id为CUSTOM_PARAM_GUEST_ID_2时有效；


		UINT8 data[508];//限定联合体为508字节长度（该字段不使用，仅用于数据结构长度定义）
	}uParam;
}GuestCustomParam;


typedef struct tagRoiItem
{
	UINT32  left;//起始列，从0开始;
	UINT32  top;//起始行，从0开始;
	UINT32  right;//终止列，不超过图像宽;
	UINT32  bottom;//终止行，不超过图像高;

}RoiItem;


typedef struct tagDepthCalRoi
{
	RoiItem struMax;//最大值，只读
	RoiItem struDefault;//默认值，只读

	RoiItem struCurrent;//当前值，可读写

}DepthCalRoi;


//TOF模组内参和畸变（通用模型）
typedef struct tagTofModuleLensGeneral
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
}TofModuleLensGeneral;

//TOF模组内参和畸变（鱼眼模型）
typedef struct tagTofModuleLensFishEye
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 k3;
	FLOAT32 k4;
}TofModuleLensFishEye;

//TOF模组内参和畸变（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
typedef struct tagTofModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}TofModuleLensParameter;

//TOF模组内参和畸变（V2.0版本）
typedef struct tagTofModuleLensParameterV20
{
	UINT32 nIndex;//1---general有效, 2---fishEye有效

	union
	{
		//[第1种]: 普通模型
		TofModuleLensGeneral general;//普通模型

		//[第2种]: 鱼眼模型
		TofModuleLensFishEye fishEye;//鱼眼模型
	}uParam;

}TofModuleLensParameterV20;

//TOF模组标定数据
typedef struct tagTofCalibData
{
	UINT8* pData;//指向标定数据
	UINT32 nDataLen;//pData内标定数据长度
}TofCalibData;

typedef struct tagTofRawData
{
	//RAW数据
	UINT8* pRaw;//一帧RAW数据
	UINT32 nRawLen;//RAW数据长度（字节数）

	//RAW数据其他属性参数
	FLOAT32 fTemperature;//出RAW数据时模组温度（注意：部分型号模组不需要该字段、部分模组RAW数据自带该数据，那么可以输入0值）

}TofRawData;


typedef struct tagExterntionHooks
{
	void* pUserData;//用户自定义数据

	/**********用于提前送出计算出来的曝光值**********/
	//@    pExp:  计算出的曝光值信息;
	//@    user_data:  用户自定义数据，与pUserData属于同一个;
	//@    【特别注意】:  对于在该回调函数内调用TOFM_XXX接口时，只允许调用软件算法部分接口，否则会死锁！！！！！
	void(*RecvTofExpTime)(TofExpouseCurrentItems* pExp, void*user_data);//根据模组实际情况选择是否实现


}ExterntionHooks;



#endif //__TYPEDEF_H__


