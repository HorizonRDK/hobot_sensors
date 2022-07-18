#ifndef __COMM_TYPEDEF_H__
#define __COMM_TYPEDEF_H__

typedef struct RawDataInfo
{
	unsigned long long timeStamp;//时间戳
	unsigned char *pucRawData; // 输出的RAW数据地址
	unsigned int uiRawSize;// 输出的RAW数据长度
	unsigned int uiFrameCnt;// 帧计数
} RAW_DATA_INFO_S;


typedef enum PixelFormat
{
	PIXEL_FORMAT_NONE = -1,
	PIXEL_FORMAT_RAW,
	PIXEL_FORMAT_YUV,
	PIXEL_FORMAT_RGB,
	PIXEL_FORMAT_MJPEG,
	PIXEL_FORMAT_H264,
	PIXEL_FORMAT_TOTAL,
} SY_PIXEL_FORMAT_E;


typedef struct ImageDataInfo
{
	SY_PIXEL_FORMAT_E ePixelFormat;
	unsigned long long timeStamp;//时间戳
	unsigned char *pucImageData; // 输出的图像数据地址
	unsigned int uiImageSize;// 输出的图像数据长度
	unsigned int uiFrameCnt;// 帧计数
} IMAGE_DATA_INFO_S;

#endif

