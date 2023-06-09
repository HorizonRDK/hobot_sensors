#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>

#endif
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include "tof_error.h"
#include "tof_mod_sdk.h"

#include "tof_depth_process.h"
#include "tof_i2c.h"
#include "camera_control.h"
#include "camera_hb_cfg.h"

//
#define SAFE_DELETE(p) if(p){delete p; p=NULL;}
#define SAFE_DELETE_ARRY(p) if(p){delete [] p; p=NULL;}
#define SAFE_FREE(p) if(p){free(p); p=NULL;}
#define SAFE_CLOSE_FP(fp) if(fp){fclose(fp); fp=NULL;}
#define LINUX


#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);
		
	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);
	
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec/1000000);
	
#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static long Utils_GetFileLen(const char * filename)
{
	if (NULL == filename)
	{
		printf("NULL == filename\n");
		return 0;
	}

	FILE * fd = fopen(filename, "rb");
	if (NULL == fd)
	{
		printf("open file (%s) failed, errno=%d(%s).\n", filename, errno, strerror(errno));
		return 0;
	}

	fseek(fd, 0L, SEEK_END); /* 定位到文件末尾 */
	const long len = ftell(fd);
	fclose(fd);

	return len;

}

static void Utils_SaveBufToFile(void* pData, const unsigned int nDataLen, const char* pFile, const bool bAppend)
{
	if ((NULL == pData) || (0 >= nDataLen) || (NULL == pFile))
	{
		return;
	}

	FILE* fp = fopen(pFile, (bAppend ? "ab" : "wb"));
	if (NULL == fp)
	{
		printf("open file(%s) failed, error=%d(%s).\n", pFile, errno, strerror(errno));
		return;
	}

	fwrite(pData, 1, nDataLen, fp);
	fclose(fp);
}

template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

template <class T>
T Utils_FindMinValue(T* pData, const int nCnt)
{
	T min = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (min > pData[i])
		{
			min = pData[i];
		}
	}

	return min;
}

int CaculateGrayPixelBytes(const GRAY_FORMAT format)
{
	int grayByte = 1;

	switch (format)
	{
	case GRAY_FORMAT_UINT8: grayByte = 1; break;
	case GRAY_FORMAT_UINT16: grayByte = 2; break;
	case GRAY_FORMAT_FLOAT: grayByte = 4; break;
	case GRAY_FORMAT_BGRD: grayByte = 4; break;
	default:break;
	}

	return grayByte;

}
static const char* StringGrayFormat(const GRAY_FORMAT format)
{
	const char* pStr = "UnknownGRAY";

	switch (format)
	{
	case GRAY_FORMAT_UINT8: pStr = "U8"; break;
	case GRAY_FORMAT_UINT16: pStr = "U16"; break;
	case GRAY_FORMAT_FLOAT: pStr = "FLOAT"; break;
	case GRAY_FORMAT_BGRD: pStr = "BGRD"; break;

	default: break;
	}

	return pStr;
}
static float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	default: break;
	}

	return pStr;
}


class CGrayConvert
{
	CGrayConvert();
	~CGrayConvert();

public:
	static bool Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32);
	static bool Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16);
	static bool Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8);

private:
	static bool ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32);
	static bool ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32);
private:
	static bool ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(float* pGray, const int width, const int height, unsigned short* pU16);
	static bool ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16);
private:
	static bool ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(float* pGray, const int width, const int height, unsigned char* pU8);
	static bool ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8);
};
CGrayConvert::CGrayConvert()
{
}
CGrayConvert::~CGrayConvert()
{
}

bool CGrayConvert::Gray_2_Bgr32(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned int* pBgr32)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToBgr32((unsigned char*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_UINT16: retVal = ToBgr32((unsigned short*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToBgr32((float*)pGray, width, height, pBgr32); break;
	case GRAY_FORMAT_BGRD:   retVal = ToBgr32((unsigned int*)pGray, width, height, pBgr32); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U16(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned short* pU16)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU16((unsigned char*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_UINT16: retVal = ToU16((unsigned short*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU16((float*)pGray, width, height, pU16); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU16((unsigned int*)pGray, width, height, pU16); break;
	default: break;
	}

	return retVal;
}

bool CGrayConvert::Gray_2_U8(void* pGray, const int width, const int height, const GRAY_FORMAT format, unsigned char* pU8)
{
	bool retVal = false;

	switch (format)
	{
	case GRAY_FORMAT_UINT8:  retVal = ToU8((unsigned char*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_UINT16: retVal = ToU8((unsigned short*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_FLOAT:  retVal = ToU8((float*)pGray, width, height, pU8); break;
	case GRAY_FORMAT_BGRD:   retVal = ToU8((unsigned int*)pGray, width, height, pU8); break;
	default: break;
	}

	return retVal;
}
bool CGrayConvert::ToBgr32(unsigned char* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pBgr32[i] = ((pGray[i] << 24) | (pGray[i] << 16) | (pGray[i] << 8) | pGray[i]);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned short* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}

	return true;
}
bool CGrayConvert::ToBgr32(float* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pBgr32, 0, pixel_cnt * sizeof(pBgr32[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pBgr32[i] = ((tmp << 24) | (tmp << 16) | (tmp << 8) | tmp);
	}

	return true;
}
bool CGrayConvert::ToBgr32(unsigned int* pGray, const int width, const int height, unsigned int* pBgr32)
{
	const int pixel_cnt = width*height;

	memcpy(pBgr32, pGray, pixel_cnt * sizeof(pBgr32[0]));

	return true;
}

bool CGrayConvert::ToU16(unsigned char* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;
	//const unsigned char min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned char max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0 / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned short tmp = (unsigned short)(pGray[i] * K);
		pU16[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU16(unsigned short* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	memcpy(pU16, pGray, pixel_cnt * sizeof(pU16[0]));

	return true;
}
bool CGrayConvert::ToU16(float* pGray, const int width, const int height, unsigned short* pU16)
{
#if 1
	//方法1：灰度直接数据类型强转

	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		pU16[i] = ((65535.0 < pGray[i]) ? 65535 : pGray[i]);
	}

#else
	//方法2：灰度按照等比例压缩

	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pU16, 0, pixel_cnt * sizeof(pU16[0]));
		return true;
	}

	const float K = (65535 * 1.0 / max);//最大值是65535的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned short tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned short)(pGray[i] * K);
		}
		pU16[i] = tmp;
	}
#endif
	return true;
}
bool CGrayConvert::ToU16(unsigned int* pGray, const int width, const int height, unsigned short* pU16)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU16[i] = (pTmp[0] << 8);//放大到65535
	}

	return true;
}

bool CGrayConvert::ToU8(unsigned char* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	memcpy(pU8, pGray, pixel_cnt * sizeof(pU8[0]));

	return true;
}
bool CGrayConvert::ToU8(unsigned short* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;
	//const unsigned short min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const unsigned short max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		const unsigned char tmp = (unsigned char)(pGray[i] * K);
		pU8[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU8(float* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}

	return true;
}
bool CGrayConvert::ToU8(unsigned int* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char* pTmp = (unsigned char*)(pGray + i);//BGRD排列
		pU8[i] = pTmp[0];
	}

	return true;
}

static bool SaveGray_2_BGR32(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned int* pData = new unsigned int[width * height];//bgra
	CGrayConvert::Gray_2_Bgr32(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}
static bool SaveGray_2_U16(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned short* pData = new unsigned short[width * height];//U16
	CGrayConvert::Gray_2_U16(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

bool TofGray2U8(void* pGrayData, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, void ** pOutGrayU8)
{
	if ((NULL == pGrayData) || (0 >= width) || (0 >= height) )
	{
		return false;
	}
	unsigned char* pData = new unsigned char[width * height];  // U8
	CGrayConvert::Gray_2_U8(pGrayData, width, height, format, pData);
	*pOutGrayU8 = (void*)pData;
}

static bool SaveGray_2_U8(void* pGray, const UINT32 width, const UINT32 height, const GRAY_FORMAT format, char* pFile)
{
	if ((NULL == pGray) || (0 >= width) || (0 >= height) || (NULL == pFile))
	{
		return false;
	}

	unsigned char* pData = new unsigned char[width * height];//U8
	CGrayConvert::Gray_2_U8(pGray, width, height, format, pData);
	Utils_SaveBufToFile(pData, width * height * sizeof(pData[0]), (const char*)pFile, false);
	SAFE_DELETE_ARRY(pData);

	return true;

}

static bool SaveDepthText(float* pDepthData, const UINT32 width, const UINT32 height, char* pTxtFile, const bool bWH)
{
	if ((NULL == pDepthData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	if (bWH)//1：W列、H行排列
	{
		UINT32 nPos = 0;
		for (UINT32 h = 0; h < height; h++)
		{
			for (UINT32 w = 0; w < (width - 1); w++)
			{
				fprintf(fp, "%0.6f,", pDepthData[nPos]);
				nPos++;
			}
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
			nPos++;
		}
	}
	else//2：1行、W*H行排列
	{
		const UINT32 nCnt = width *height;
		for (UINT32 nPos = 0; nPos < nCnt; nPos++)
		{
			fprintf(fp, "%0.6f\n", pDepthData[nPos]);
		}
	}

	fclose(fp);
	return true;
}
static bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;%0.6f;%0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z);
	}

	fclose(fp);
	return true;
}
static bool SavePointDataZWHText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	UINT32 nPos = 0;
	for (UINT32 h = 0; h < height; h++)
	{
		for (UINT32 w = 0; w < width; w++)
		{
			fprintf(fp, "%0.6f", pPointData[nPos].z);
			nPos++;
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}

static bool SaveFloatPointDataText(FLOAT32 *pfPoint, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pfPoint) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%0.6f;\n", pfPoint[nPos]);
	}

	fclose(fp);
	return true;
}

static bool SaveU16PointDataText(UINT16 *pu16Point, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == pu16Point) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const UINT32 nCnt = width *height;
	for (UINT32 nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%u;\n", pu16Point[nPos]);
	}

	fclose(fp);
	return true;
}



class CBuf
{
public:
	CBuf(const unsigned int nBufLen = 128/*字节数*/)
	{
		m_nBufLen = (0 < nBufLen) ? nBufLen : 128;//防止参数错误
		m_pBuf = new unsigned char[m_nBufLen];
		m_nDataLen = 0;

		memset(m_pBuf, 0, m_nBufLen);
	}
	virtual ~CBuf()
	{
		if (m_pBuf)
		{
			delete[] m_pBuf;
			m_pBuf = NULL;
		}
	}

public:
	unsigned char* GetBuf(void)
	{
		return m_pBuf;
	}
	unsigned int GetBufLen(void)
	{
		return m_nBufLen;
	}
	unsigned int GetDataLen(void)
	{
		return m_nDataLen;
	}
	unsigned int SetDataLen(const unsigned int nDataLen)
	{
		m_nDataLen = ((nDataLen > m_nBufLen) ? m_nBufLen : nDataLen);//不允许超过缓冲区长度

		return m_nDataLen;
	}
	void ReSize(const unsigned int nBufLen)
	{
		if (m_pBuf)
		{
			delete[] m_pBuf;
			m_pBuf = NULL;
		}

		m_nBufLen = (0 < nBufLen) ? nBufLen : 128;//防止参数错误
		m_pBuf = new unsigned char[m_nBufLen];
		m_nDataLen = 0;

		memset(m_pBuf, 0, m_nBufLen);
	}


private:
	unsigned char* m_pBuf;//指向一块缓冲区
	unsigned int m_nBufLen;//m_pBuf长度
	unsigned int m_nDataLen;//m_pBuf内数据长度

};


class CTofModuleHalUserData
{
public:
	CTofModuleHalUserData(const char* pCalibFile)
	{
		m_strCalibFile = pCalibFile;
		if (NULL == (m_fpCalibFile = fopen(pCalibFile, "rb")))
		{
			printf("open calib file %s failed.\n:", pCalibFile);
		}
	}
	virtual ~CTofModuleHalUserData()
	{
		m_strCalibFile = "";
		if (m_fpCalibFile)
		{
			fclose(m_fpCalibFile);
			m_fpCalibFile = NULL;
		}
	}

public:
	FILE* GetCalibFile(void)
	{
		return m_fpCalibFile;
	}

private:
	std::string m_strCalibFile;
	FILE* m_fpCalibFile;

};


static SBOOL TofModuleHal_Init(void* user_data)
{
	//anything you can do, by youself

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;


	return true;
}
static SBOOL TofModuleHal_Deinit(void* user_data)
{
	//anything you can do, by youself

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;


	return true;
}

static SBOOL TofModuleHal_WriteReg16(const UINT8 slave_addr, const UINT16 regAddr, const UINT16 value, void*user_data)
{
	//generally , it is used to write a value to a sensor register
	I2C_CB_S *pstI2cCb = (I2C_CB_S*)user_data;

	I2cWrite(pstI2cCb, regAddr, value, I2C_FMT_A16D16);

	return true;
}

static SBOOL TofModuleHal_ReadReg16(const UINT8 slave_addr, const UINT16 regAddr, UINT16 *value, void*user_data)
{
	//generally , it is used to read  a value from a sensor register
	I2C_CB_S *pstI2cCb = (I2C_CB_S*)user_data;

	I2cRead(pstI2cCb, regAddr, value, I2C_FMT_A16D16);

	return true;
}

static UINT32 TofModuleHal_ReadRegBulk(const UINT8 slave_addr, const UINT32 startAddr, const UINT32 addrCnt, void *pBufOut, void*user_data)
{
	//generally , it is used to read calib data from eeprom
	//but, in some modules(without eeprom, but with spi flash), it is not used.

	CTofModuleHalUserData*pInput = (CTofModuleHalUserData*)user_data;
	if (pInput)
	{
		return ((UINT32)fread(pBufOut, 1, addrCnt, pInput->GetCalibFile()));
	}
	return 0;
}

static void ExterntionHooks_RecvTofExpTime(TofExpouseCurrentItems* pExp, void*user_data)
{
	if (1 == pExp->nIndex)
	{
		printf("RecvTofExpTime: time:%llu, exp:%d.\n", Utils_GetTickCount(), pExp->uParam.g1.exp);
	}
	else if (2 == pExp->nIndex)
	{
		printf("RecvTofExpTime: time:%llu, exp_AEF:%d, exp_FEF:%d.\n", Utils_GetTickCount(), pExp->uParam.g2.exp_AEF, pExp->uParam.g2.exp_FEF);
	}
	else
	{
		printf("ExterntionHooks RecvTofExpTime: time:%u, not supported.\n", pExp->nIndex);
		return;
	}

	HTOFM hDev = (HTOFM)user_data;
	const TOFRET retVal = TOFM_SetTofExpTime(hDev, pExp);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("ExterntionHooks: TOFM_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	}
}


static SBOOL ReadFile(std::string& strFile, CBuf& buf)
{
	const long file_len = Utils_GetFileLen(strFile.c_str());
	if (0 >= file_len)
	{
		printf("read file(%s), failed.......\n", strFile.c_str());
		return false;
	}

	if (file_len > (const long)buf.GetBufLen())
	{
		buf.ReSize(file_len);
	}

	FILE* fp = fopen(strFile.c_str(), "rb");
	const size_t read_len = fread(buf.GetBuf(), 1, file_len, fp);
	fclose(fp);

	buf.SetDataLen((const unsigned int)read_len);

	printf("read file(%s), ok, file_len=%ld, read_len=%lu.......\n", strFile.c_str(), file_len, read_len);

	return true;
}

static void CaptureTofFrame(const std::string& strDir, const std::string& strTofName, const unsigned int nCaptureIndex, TOF_DEPTH_DATA_INFO_S *tofFrameData)
{
	const unsigned int nPixelCnt = tofFrameData->frameWidth * tofFrameData->frameHeight;
	char szFile[512] = { 0 };

	//
	if (NULL != tofFrameData->pDepthData)
	{
		sprintf(szFile, "%s/%s-%u-DepthData.dat", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pDepthData, nPixelCnt  * sizeof(tofFrameData->pDepthData[0]), szFile, false);

		sprintf(szFile, "%s/%s-%u-DepthData.txt", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		SaveDepthText(tofFrameData->pDepthData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile, false);
	}

	//
	if (NULL != tofFrameData->pfPointData)
	{
		sprintf(szFile, "%s/%s-%u-PointData.dat", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		Utils_SaveBufToFile(tofFrameData->pfPointData, nPixelCnt  * sizeof(tofFrameData->pfPointData[0]), szFile, false);

		sprintf(szFile, "%s/%s-%u-PointData.txt", strDir.c_str(), strTofName.c_str(), nCaptureIndex);
		SavePointDataXYZText(tofFrameData->pfPointData, tofFrameData->frameWidth, tofFrameData->frameHeight, szFile);
	}

	//
	if (NULL != tofFrameData->pGrayData)
	{
		sprintf(szFile, "%s/%s-%u-Gray.%s", strDir.c_str(), strTofName.c_str(), nCaptureIndex, StringGrayFormat(tofFrameData->grayFormat));
		Utils_SaveBufToFile(tofFrameData->pu16GrayData, nPixelCnt * CaculateGrayPixelBytes(tofFrameData->grayFormat), szFile, false);

		//sprintf(szFile, "%s/%u-Gray.u8", strDir.c_str(), nCaptureIndex);
		//SaveGray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, szFile);
	}
	//
	/*
	if ((NULL != tofFrameData->pExtData) && (0 < tofFrameData->nExtDataLen))
	{
		sprintf(szFile, "%s/%u-Tof.%s", strDir.c_str(), nCaptureIndex, "extdata");
		Utils_SaveBufToFile(tofFrameData->pExtData, tofFrameData->nExtDataLen, szFile, false);
	}
	*/
}

typedef struct tagSunnyConciseDepthData
{
	unsigned short* u16Depth;
	unsigned char* u8Gray;
}SunnyConciseDepthData;

typedef struct tagSunnyFullDepthData
{
	unsigned short* u16Depth;
	unsigned char* u8Gray;
	unsigned char* u8Intensity;
	unsigned short* u16Confidence;
}SunnyFullDepthData;

//安克客户MTP013时候额外需要的数据
typedef struct tagAnkerMtp013CustomExtData
{
	float* pConfidence;
	float* intensity;
}AnkerMtp013CustomExtData;

static void SunnyConciseDepthDataGet(TofModDepthData *tofFrameData, SunnyConciseDepthData *depthData)
{
	const UINT32 pixle_cnt = tofFrameData->frameWidth * tofFrameData->frameHeight;

	//u16Depth
	for (UINT32 pos = 0; pos < pixle_cnt; pos++)
	{
		depthData->u16Depth[pos] = tofFrameData->pPointData[pos].z * 1000 + 0.5;
	}

	//u8Gray
	CGrayConvert::Gray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, depthData->u8Gray);

}

static void SunnyFullDepthDataGet(TofModDepthData *tofFrameData, SunnyFullDepthData *depthData)
{
	const UINT32 pixle_cnt = tofFrameData->frameWidth * tofFrameData->frameHeight;

	//u16Depth
	for (UINT32 pos = 0; pos < pixle_cnt; pos++)
	{
		depthData->u16Depth[pos] = tofFrameData->pPointData[pos].z * 1000 + 0.5;
	}

	//u8Gray
	CGrayConvert::Gray_2_U8(tofFrameData->pGrayData, tofFrameData->frameWidth, tofFrameData->frameHeight, tofFrameData->grayFormat, depthData->u8Gray);

	if ((NULL != tofFrameData->pExtData) && (sizeof(AnkerMtp013CustomExtData) == tofFrameData->nExtDataLen))
	{	
		AnkerMtp013CustomExtData* pExt = (AnkerMtp013CustomExtData*)(tofFrameData->pExtData);

		//u8Intensity, u16Confidence
		for (UINT32 pos = 0; pos < pixle_cnt; pos++)
		{
			depthData->u8Intensity[pos]   = ((255 > pExt->intensity[pos]) ? pExt->intensity[pos] : 255);//超过255取255,否则强转
			depthData->u16Confidence[pos] = ((0 > pExt->pConfidence[pos]) ? 0 : (pExt->pConfidence[pos] * 1000));//小于0取0,否则放大1000倍强转
		}
	}

}


void HandleDepthData(const UINT32 threadIndex, UINT32 frameIndex, std::string& strSaveDir, std::string strTofName, TOF_DEPTH_DATA_INFO_S* tofFrameData)
{
	//todo anthing by youself. for example, save to file:

	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	const float fDepthZAvg = CalCenterPointDataZAvg(tofFrameData->pfPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	printf("[%u], one TOF frame,time=%llu, center depthZ = %0.3f m.\n", threadIndex, Utils_GetTickCount(), fDepthZAvg);

	CaptureTofFrame(strSaveDir, strTofName, frameIndex, tofFrameData);
}

#ifdef RGBD
static bool SaveColorPointDataXYZText(TofRgbdPointCloudColor *pPointData, const unsigned int width, const unsigned int height, char* pTxtFile)
{
	if ((NULL == pPointData) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const unsigned int nCnt = width *height;
	for (unsigned int nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "v %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", pPointData[nPos].x, pPointData[nPos].y, pPointData[nPos].z, pPointData[nPos].r, pPointData[nPos].g, pPointData[nPos].b);
	}

	fclose(fp);
	return true;
}

static bool SavePixelCoordText(TofRgbdPixelCoord *pPixelCoord, const unsigned int width, const unsigned int height, char* pTxtFile)
{
	if ((NULL == pPixelCoord) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	const unsigned int nCnt = width *height;
	for (unsigned int nPos = 0; nPos < nCnt; nPos++)
	{
		fprintf(fp, "%u %u\n", pPixelCoord[nPos].x, pPixelCoord[nPos].y);
	}

	fclose(fp);
	return true;
}

unsigned int ConvertPixelCoordMap2Rgb(unsigned char* pRgb, unsigned int nRgbWidth, unsigned int nRgbHeight, TofRgbdPixelCoord* pCoord, unsigned int nCoordWidth, unsigned int nCoordHeight, unsigned char* pOutRgb)
{
	for (int i = 0; i < nCoordWidth *nCoordHeight; i++)
	{
		unsigned int rgbpixle = pCoord[i].x + pCoord[i].y * nRgbWidth;
		if (0 != rgbpixle)
		{
			memcpy(pOutRgb + 3 * i, pRgb + 3 * rgbpixle, 3);//目前只考虑3通道rgb的情况
		}
	}

	return (nCoordWidth * nCoordHeight * 3);
}

static void CaptureTofRgbdOutputData(const std::string& strDir, const unsigned int nCaptureIndex, TofRgbdInputData* pDataIn, TofRgbdOutputData *rgbdData)
{
	char szFile[512] = { 0 };

	//
	if (1)
	{
		TofRgbdImage_PointCloud& tmp = rgbdData->pointCloud2Rgb;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-pointCloud2Rgb.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-pointCloud2Rgb.txt", strDir.c_str(), nCaptureIndex);
			SavePointDataXYZText((PointData*)tmp.pData, tmp.nWidth, tmp.nHeight, szFile);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_U8& tmp = rgbdData->gray2Rgb;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-gray2Rgb.u8", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_Void& tmp = rgbdData->rgb2Tof;
		if ((NULL != tmp.pData) && (0 < tmp.nDataLen))
		{
			sprintf(szFile, "%s/%u-rgb2Tof.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, tmp.nDataLen, szFile, false);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_PointCloudColor& tmp = rgbdData->colorPointCloud;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-colorPointCloud.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-colorPointCloud.obj", strDir.c_str(), nCaptureIndex);
			SaveColorPointDataXYZText(tmp.pData, tmp.nWidth, tmp.nHeight, szFile);
		}
	}

	//
	if (1)
	{
		TofRgbdImage_PixelCoord& tmp = rgbdData->rgb2TofPixelCoord;
		if ((NULL != tmp.pData) && (0 < tmp.nWidth) && (0 < tmp.nHeight))
		{
			const unsigned int nPixelCnt = tmp.nWidth * tmp.nHeight;

			sprintf(szFile, "%s/%u-rgb2TofPixelCoord.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, nPixelCnt  * sizeof(tmp.pData[0]), szFile, false);

			sprintf(szFile, "%s/%u-rgb2TofPixelCoord.txt", strDir.c_str(), nCaptureIndex);
			SavePixelCoordText(tmp.pData, tmp.nWidth, tmp.nHeight, szFile);


			CBuf bufRgb(tmp.nWidth * tmp.nHeight * 3);//目前只考虑3通道rgb的情况
			const unsigned int nRetLen = ConvertPixelCoordMap2Rgb(pDataIn->pRgb, RGB_WIDTH, RGB_HEIGHT, tmp.pData, tmp.nWidth, tmp.nHeight, bufRgb.GetBuf());
			sprintf(szFile, "%s/%u-rgb2TofPixelCoord-rgb.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(bufRgb.GetBuf(), nRetLen, szFile, false);

		}
	}


	//
	if (1)
	{
		TofRgbdImage_Priv& tmp = rgbdData->privData;
		if ((NULL != tmp.pData) && (0 < tmp.nDataLen))
		{
			sprintf(szFile, "%s/%u-privData.dat", strDir.c_str(), nCaptureIndex);
			Utils_SaveBufToFile(tmp.pData, tmp.nDataLen, szFile, false);
		}
	}
}

void HandleTofRgbdOutputData(unsigned int frameIndex, const std::string& strSaveDir, TofRgbdInputData* pDataIn, TofRgbdOutputData* rgbdData)
{
	//todo anthing by youself. for example, save to file:

	CaptureTofRgbdOutputData(strSaveDir, frameIndex, pDataIn, rgbdData);
}
#endif

static void AlsoCanGetOrSetSomeParam(HTOFM hTofMod, TofModuleCaps* pCaps)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	//TofExpouseCurrentItems struExpCurrent;
	//memset(&struExpCurrent, 0, sizeof(struExpCurrent));
	//struExpCurrent.nIndex = 1;
	////struExpCurrent.nIndex = 2;
	//if (1 == struExpCurrent.nIndex)
	//{
	//	struExpCurrent.uParam.g1.exp = 100;
	//}
	//else if (2 == struExpCurrent.nIndex)
	//{
	//	struExpCurrent.uParam.g2.exp_AEF = 200;
	//	struExpCurrent.uParam.g2.exp_FEF = 100;
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofExpTime(hTofMod, &struExpCurrent)))
	//{
	//	printf("TOFM_SetTofExpTime failed, retVal=0x%08x.\n", retVal);
	//}
	//TofExpouseItems struExp;
	//memset(&struExp, 0, sizeof(struExp));
	//if (TOFRET_SUCCESS != (retVal = TOFM_GetTofExpTime(hTofMod, &struExp)))
	//{
	//	printf("TOFM_GetTofExpTime failed, retVal=0x%08x.\n", retVal);
	//}
	//else
	//{
	//	if (1 == struExp.nIndex)
	//	{
	//		printf("TOFM_GetTofExpTime, exp=%d.\n", struExp.uParam.g1.exp);
	//	}
	//	else if (2 == struExp.nIndex)
	//	{
	//		printf("TOFM_GetTofExpTime, exp_AEF=%d, exp_FEF=%d.\n", struExp.uParam.g2.exp_AEF, struExp.uParam.g2.exp_FEF);
	//	}
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofFilter(hTofMod, const TOF_FILTER type, true)))
	//{
	//	printf("TOFM_SetTofFilter failed, retVal=0x%08x.\n", retVal);
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_GetTofFilter(hTofMod, const TOF_FILTER type, SBOOL* pbEnable)))
	//{
	//	printf("TOFM_GetTofFilter failed, retVal=0x%08x.\n", retVal);
	//}
	//if (TOFRET_SUCCESS != (retVal = TOFM_SetTofHDRZ(hTofMod, false)))
	//{
	//	printf("TOFM_SetTofHDRZ failed, retVal=0x%08x.\n", retVal);
	//}
}

static void GetOrSetSomeParam(HTOFM hTofMod, TofModuleCaps* pCaps, const TOF_MODE tofMode)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	for (UINT32 i = 0; i < 32; i++)
	{
		UINT32 type = (1 << i);
		if (0 != (pCaps->supportedTOFFilter & type))
		{
			if (TOFRET_SUCCESS != (retVal = TOFM_SetTofFilter(hTofMod, (const TOF_FILTER)type, true)))
			{
				printf("TOFM_SetTofFilter(0x%08x) failed, retVal=0x%08x.\n", (TOF_FILTER)type, retVal);
			}
		}
	}

	if (pCaps->bTofRemoveINSSupported)
	{
		bool bEnable = false;
		if ((TOF_MODE_HDRZ_5FPS == tofMode) || (TOF_MODE_HDRZ_10FPS == tofMode)
			|| (TOF_MODE_HDRZ_15FPS == tofMode) || (TOF_MODE_HDRZ_30FPS == tofMode)
			|| (TOF_MODE_HDRZ_45FPS == tofMode) || (TOF_MODE_HDRZ_60FPS == tofMode))
		{	//注意：目前只有17相位的才调试过效果，所以开放，其他相位的效果没调试过，所以不开放
			bEnable = true;
		}
		if (TOFRET_SUCCESS != (retVal = TOFM_SetTofRemoveINS(hTofMod, bEnable)))
		{
			printf("TOFM_SetTofRemoveINS (%d) failed, retVal=0x%08x.\n", bEnable, retVal);
		}
	}
}

static void PrintfSomeCalibParam(SomeCalibParam* pParamOut)
{
	if (NULL == pParamOut)  return;

	if (2 == pParamOut->struLensParamterV20.nIndex)
	{
		printf("struLensParamter-fishEye:...............................\n");
		printf(">>   fx = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.fx);
		printf(">>   fy = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.fy);
		printf(">>   cx = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.cx);
		printf(">>   cy = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.cy);
		printf(">>   k1 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k1);
		printf(">>   k2 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k2);
		printf(">>   k3 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k3);
		printf(">>   k4 = %f.\n", pParamOut->struLensParamterV20.uParam.fishEye.k4);
	}
}

static bool ReadTemperature(HTOFM hTofMod, FLOAT32* pTemperature)
{
	FLOAT32 temperature = 0;
	const TOFRET retVal = TOFM_GetTemperature(hTofMod, &temperature);//不是所有的模组都支持这个接口
	if (TOFRET_SUCCESS == retVal)
	{
		*pTemperature = temperature;
		return true;
	}
	else if (TOFRET_ERROR_NOT_SUPPORTED == retVal)
	{
		*pTemperature = 0;
		return true;
	}
	else
	{
		printf("TOFM_GetTemperature failed, retVal=0x%08x.\n", retVal);
		return false;
	}
}

static void Sunny_RecvTofExpTime(TofExpouseCurrentItems* pExp, void *user_data)
{
	DEPTH_HANDLE_CB_S *pstDepthHandleCb = (DEPTH_HANDLE_CB_S*)user_data;

	if (1 == pExp->nIndex)
	{
		pstDepthHandleCb->stTofSetExp.expTime = pExp->uParam.g1.exp;
		pstDepthHandleCb->stTofSetExp.eExpMode = SINGLE_FRAME_MODE;
	}
	else if (2 == pExp->nIndex)
	{
		pstDepthHandleCb->stTofSetExp.expTime_AEF = pExp->uParam.g2.exp_AEF;
		pstDepthHandleCb->stTofSetExp.expTime_FEF = pExp->uParam.g2.exp_FEF;
		pstDepthHandleCb->stTofSetExp.eExpMode = LONG_SHORT_FRAME_MODE;
		
		//printf("start tof-%s AEF[%d] FEF[%d]\n", pstCameraHandler->acDirection, pExp->uParam.g2.exp_AEF, pExp->uParam.g2.exp_FEF);
	}
	
	pstDepthHandleCb->stTofSetExp.flag = START_SETUP_EXP;
}


void *Sunny_SetTofEXP(void *para)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofExpouseCurrentItems stExpCurtItems = {0};
	DEPTH_HANDLE_CB_S *pstDepthHandleCb = (DEPTH_HANDLE_CB_S*)para;
	
	while(pstDepthHandleCb->isExpTrdRunning)
	{
		if(pstDepthHandleCb->stTofSetExp.flag == START_SETUP_EXP )
		{
			memset(&stExpCurtItems, 0, sizeof(stExpCurtItems));
		
			if (pstDepthHandleCb->stTofSetExp.eExpMode == LONG_SHORT_FRAME_MODE)
			{
				stExpCurtItems.nIndex = 2;
				stExpCurtItems.uParam.g2.exp_AEF = pstDepthHandleCb->stTofSetExp.expTime_AEF;
				stExpCurtItems.uParam.g2.exp_FEF = pstDepthHandleCb->stTofSetExp.expTime_FEF;
				//printf("Set tof AEF[%d] FEF[%d]\n", pstDepthHandleCb->stTofSetExp.expTime_AEF, 
				//	pstDepthHandleCb->stTofSetExp.expTime_FEF);
			}
			else if (pstDepthHandleCb->stTofSetExp.eExpMode == SINGLE_FRAME_MODE)
			{
				stExpCurtItems.nIndex = 1;
				stExpCurtItems.uParam.g1.exp = pstDepthHandleCb->stTofSetExp.expTime;
				//printf("end EXP[%d]\n", stSetExp.expTime);
			}

			TOFM_SetTofExpTime(pstDepthHandleCb->hTofMod, &stExpCurtItems);
			pstDepthHandleCb->stTofSetExp.flag = STOP_SETUP_EXP;
		}
		
		usleep(1000);		
	}

	return NULL;
}


void TofDepthSdkGloableInit()
{

	//0. 初始化.......
	TofModuleInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, SELECT_MODULE_CFG_FILE_PATH, sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	TOFM_Init(&struInitParam);//必须放在最开始

	printf("SDK Version: %s.\n", TOFM_GetSDKVersion());
}


void TofDepthSdkGloableUnInit()
{
	TOFM_Uninit();//必须与TOFM_Init配套使用
}


int TofDepthSdkInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb, char *pcCalibDataPath, void *pHalUserData)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	TofModuleCaps stTofModuleCaps;
	CBuf calibData(512 * 1024);//不能少于128K,，一般不超过128K

	if (!pstDepthHandleCb || !pcCalibDataPath || !pHalUserData)
	{
		printf("NULL ptr!\n");
		return -1;
	}

	//1. 打开模组.......
	const std::string strModule = (SELECT_MODULE_NAME);

	const TOF_MODE tofMode = TOF_MODE_HDRZ_10FPS;
	std::string strCalibFile = pcCalibDataPath;

	TofModuleHal struHal;// 传入参数
	memset(&struHal, 0, sizeof(struHal));
	struHal.Init 		= TofModuleHal_Init;
	struHal.Deinit 		= TofModuleHal_Deinit;
	struHal.WriteReg16 	= TofModuleHal_WriteReg16;
	struHal.ReadReg16 	= TofModuleHal_ReadReg16;
	struHal.ReadRegBulk = TofModuleHal_ReadRegBulk;

	HTOFM hTofMod = TOFM_OpenDeviceV30((char*)(strModule.c_str()), SELECT_MODULE_GUEST_ID, &struHal, pHalUserData, &stTofModuleCaps);
	if (NULL == hTofMod)
	{
		printf("TOFM_OpenDevice failed.\n");
		return -1;
	}

	// 可自行指定配置文件名，为空则使用默认配置文件
	const std::string strCfgFile = ("");	
	
	SCHAR* pModCfgFile = (("" == strCfgFile) ? NULL : (SCHAR*)(strCfgFile.c_str()));
	if (TOFRET_SUCCESS != (retVal = TOFM_SetTofModeV20(hTofMod, tofMode, pModCfgFile)))
	{
		printf("TOFM_SetTofMode(tofMode:0x%08x) failed, retVal=0x%08x.\n", tofMode, retVal);
		goto errExitCloseDev;
	}

	//2. 从设备读取标定数据，可以备份着
	{
		//const UINT32 nCalibDataLen = TOFM_ReadCalibData(hTofMod, calibData.GetBuf(), calibData.GetBufLen());
		if (!ReadFile(strCalibFile, calibData))
		{
			printf("ReadCalibData failed.\n");
			remove(pcCalibDataPath);
			goto errExitCloseDev;
		}
	}

	//3. 加载并解析标定数据，可以是之前备份着的标定数据
	if (TOFRET_SUCCESS != (retVal = TOFM_LoadCalibData(hTofMod, calibData.GetBuf(), calibData.GetDataLen())))
	{
		printf("TOFM_LoadCalibData failed, retVal=0x%08x.\n", retVal);
		remove(pcCalibDataPath);
		goto errExitCloseDev;
	}

	//4. 这时候可以读取一些标定参数出来备份着
	SomeCalibParam struSomeCalibParam;
    if (TOFRET_SUCCESS == (TOFM_GetSomeCalibParam(hTofMod, &struSomeCalibParam)))
    {
        PrintfSomeCalibParam(&struSomeCalibParam);
    }

	//深度计算模块（该部分必须在TOFM_LoadCalibData之后，TOFM_UnLoadCalibData之前调用）
	//5. 初始化深度计算模块
	if (TOFRET_SUCCESS != (retVal = TOFM_InitDepthCal(hTofMod)))//(比较耗时)
	{
		printf("TOFM_InitDepthCal failed, retVal=0x%08x.\n", retVal);
		goto errExitUnLoadCalibData;
	}

	//7. 一般在开流之后，可按需获取/修改参数
	GetOrSetSomeParam(hTofMod, &stTofModuleCaps, tofMode);////ae 、 滤波

	// 7.1 记录支持的滤波类型
	pstDepthHandleCb->uiSupportedTofFilter = stTofModuleCaps.supportedTOFFilter;

	//8. 曝光处理设置
	ExterntionHooks stHooks;
	memset(&stHooks, 0x00, sizeof(ExterntionHooks));
	stHooks.pUserData = pstDepthHandleCb;
	stHooks.RecvTofExpTime = Sunny_RecvTofExpTime;
	TOFM_SetExterntionHooks(hTofMod, &stHooks);
	
	TofExpouseRangeItems stExpRangeItems;
	stExpRangeItems.nIndex = 2;
	TOFM_GetTofExpTimeRange(hTofMod, &stExpRangeItems);
	printf("max_AEF[%u], max_FEF[%u]\n", stExpRangeItems.uParam.g2.max_AEF, stExpRangeItems.uParam.g2.max_FEF);

	/* Create set tof exp thread */
	pstDepthHandleCb->stTofSetExp.flag = STOP_SETUP_EXP;
	pthread_create(&pstDepthHandleCb->stTofExpPid, NULL, Sunny_SetTofEXP, pstDepthHandleCb);
	pstDepthHandleCb->isExpTrdRunning = 1;

	pstDepthHandleCb->hTofMod = hTofMod;

	printf("TofDepthSdkInit successed\n");	
	return 0;

errExitUnInitDepthCal:
	//10. 回收深度计算模块
	retVal = TOFM_UnInitDepthCal(hTofMod);

errExitUnLoadCalibData:
	//11. 卸载之前导入并解析的标定数据，否则会内存泄漏
	retVal = TOFM_UnLoadCalibData(hTofMod);

errExitCloseDev:
	//12. 关闭模组.......
	retVal = TOFM_CloseDevice(hTofMod);

	printf("TofDepthSdkInit failed\n");
	return -1;
}


int TofDepthSdkUnInit(DEPTH_HANDLE_CB_S *pstDepthHandleCb)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	if (!pstDepthHandleCb)
	{
		printf("[%s] NULL ptr\n", __func__);
		return -1;
	}

	if (NULL != pstDepthHandleCb->hTofMod)
	{
		//10. 回收深度计算模块
		retVal = TOFM_UnInitDepthCal(pstDepthHandleCb->hTofMod);

		//11. 卸载之前导入并解析的标定数据，否则会内存泄漏
		retVal = TOFM_UnLoadCalibData(pstDepthHandleCb->hTofMod);

		//12. 关闭模组.......
		retVal = TOFM_CloseDevice(pstDepthHandleCb->hTofMod);
	}

	pstDepthHandleCb->isExpTrdRunning = 0;
	pthread_join(pstDepthHandleCb->stTofExpPid, NULL);

	return 0;
}

int TofDepthProcess(void *pCamHandle, IMAGE_DATA_INFO_S *pstTofRawDataInfo, TOF_DEPTH_DATA_INFO_S *pstTofDepthDataInfo)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	int i;
	unsigned int uiPixelNum = 0;
	float fConfidence = 0;
	TofRawData stRawData;
	TofModDepthData stTofDepthData;
	camera_handle *pstTofHandle;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;

	memset(&stRawData, 0, sizeof(stRawData));
	memset(&stTofDepthData, 0, sizeof(stTofDepthData));

	stRawData.nRawLen = pstTofRawDataInfo->uiImageSize;
	stRawData.pRaw = pstTofRawDataInfo->pucImageData;

	if (!pCamHandle || !pstTofRawDataInfo || !pstTofDepthDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstTofHandle = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstTofHandle->stDepthHandleCb[0];

	retVal = TOFM_DoDepthCal(pstDepthHandleCb->hTofMod, &stRawData, &stTofDepthData);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("TOFM_DoDepthCal failed, retVal=0x%08x.\n", retVal);
		return -1;
	}

	pstTofDepthDataInfo->timeStamp 		= pstTofRawDataInfo->timeStamp;
	pstTofDepthDataInfo->uiFrameCnt		= pstTofRawDataInfo->uiFrameCnt;
	pstTofDepthDataInfo->frameWidth 	= stTofDepthData.frameWidth;
	pstTofDepthDataInfo->frameHeight 	= stTofDepthData.frameHeight;
	pstTofDepthDataInfo->pDepthData		= stTofDepthData.pDepthData;
	pstTofDepthDataInfo->pfPointData 	= stTofDepthData.pPointData;
	pstTofDepthDataInfo->grayFormat		= stTofDepthData.grayFormat;
	pstTofDepthDataInfo->pGrayData		= stTofDepthData.pGrayData;

	pstTofDepthDataInfo->pu16GrayData	= NULL;
	pstTofDepthDataInfo->pu16Confidence = NULL;
	pstTofDepthDataInfo->pfNoise 		= NULL;

	return 0;
}



int TofDepthProcessExp(void *pCamHandle, IMAGE_DATA_INFO_S *pstTofRawDataInfo, TOF_DEPTH_DATA_INFO_S *pstTofDepthDataInfo, int iDepthHandleIndex)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;
	int i;
	unsigned int uiPixelNum = 0;
	float fConfidence = 0;
	TofRawData stRawData;
	TofModDepthData stTofDepthData;
	camera_handle *pstTofHandle;
	DEPTH_HANDLE_CB_S *pstDepthHandleCb;

	memset(&stRawData, 0, sizeof(stRawData));
	memset(&stTofDepthData, 0, sizeof(stTofDepthData));

	stRawData.nRawLen = pstTofRawDataInfo->uiImageSize;
	stRawData.pRaw = pstTofRawDataInfo->pucImageData;

	if (!pCamHandle || !pstTofRawDataInfo || !pstTofDepthDataInfo)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstTofHandle = (camera_handle*)pCamHandle;
	pstDepthHandleCb = &pstTofHandle->stDepthHandleCb[iDepthHandleIndex];

	retVal = TOFM_DoDepthCal(pstDepthHandleCb->hTofMod, &stRawData, &stTofDepthData);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("TOFM_DoDepthCal failed, retVal=0x%08x.\n", retVal);
		return -1;
	}

	pstTofDepthDataInfo->timeStamp 		= pstTofRawDataInfo->timeStamp;
	pstTofDepthDataInfo->uiFrameCnt		= pstTofRawDataInfo->uiFrameCnt;
	pstTofDepthDataInfo->frameWidth 	= stTofDepthData.frameWidth;
	pstTofDepthDataInfo->frameHeight 	= stTofDepthData.frameHeight;
	pstTofDepthDataInfo->pDepthData		= stTofDepthData.pDepthData;
	pstTofDepthDataInfo->pfPointData 	= stTofDepthData.pPointData;
	pstTofDepthDataInfo->grayFormat		= stTofDepthData.grayFormat;
	pstTofDepthDataInfo->pGrayData		= stTofDepthData.pGrayData;

	pstTofDepthDataInfo->pu16GrayData	= NULL;
	pstTofDepthDataInfo->pu16Confidence = NULL;
	pstTofDepthDataInfo->pfNoise 		= NULL;

	return 0;
}


#ifdef RGBD
static void PrintfLensParameter(TofRgbdLensParameter* pTofLens)
{
	if (NULL == pTofLens)  return;

	const unsigned int nIndex = pTofLens->nIndex;

	if (1 == nIndex)
	{
		TofRgbdLensGeneral* pTmp = &(pTofLens->uParam.general);

		printf("Lens Paramter (general):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   p1 = %f.\n", pTmp->p1);
		printf(">>   p2 = %f.\n", pTmp->p2);
		printf(">>   k3 = %f.\n", pTmp->k3);
	}
	else if (2 == nIndex)
	{
		TofRgbdLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		printf("Lens Paramter (fishEye):...............................\n");
		printf(">>   fx = %f.\n", pTmp->fx);
		printf(">>   fy = %f.\n", pTmp->fy);
		printf(">>   cx = %f.\n", pTmp->cx);
		printf(">>   cy = %f.\n", pTmp->cy);
		printf(">>   k1 = %f.\n", pTmp->k1);
		printf(">>   k2 = %f.\n", pTmp->k2);
		printf(">>   k3 = %f.\n", pTmp->k3);
		printf(">>   k4 = %f.\n", pTmp->k4);
	}
	else
	{
		printf("Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
	}
}

int TofRgbdSdkInit(HTOFRGBD *ppstRgbdHandle, const char *pcRgbdCalibPath)
{
	TofRgbdParameter struParameters;
	TOFRGBDRET retVal = TOFRGBDRET_FAILED;
	const std::string strModuleName = SELECT_MODULE_NAME;//模组型号
	const TOFRGBD_GUEST_ID guestID = TOFRGBD_GUEST_ID_DEF;//客户识别号

	const unsigned int nTofWidth = RAW_WIDTH;//TOF数据宽
	const unsigned int nTofHeight = ACTIVE_HEIGHT;//TOF数据高
	const unsigned int nRgbWidth = RGB_WIDTH;//RGB数据宽
	const unsigned int nRgbHeight = RGB_HEIGHT;//RGB数据高

	const TofRgbd_Gray_Format inGrayFormat = TofRgbd_Gray_Format_Float;//输入的灰度格式

	if (!ppstRgbdHandle || !pcRgbdCalibPath)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}
	
	printf("SDK Version: %s.\n", TOFRGBD_GetSDKVersion());

	CBuf calibData, pointCloudData, grayData, rgbData;
	std::string strCalibFile = pcRgbdCalibPath;
	
	if (!ReadFile(strCalibFile, calibData))
	{	printf("[%s] Read RGBD calib data failed!, path = %s\n", __func__, pcRgbdCalibPath);
		return -1;
	}
	
	TofRgbdHandleParam struInputParam;
	memset(&struInputParam, 0, sizeof(struInputParam));
	strncpy(struInputParam.szModuleName, strModuleName.c_str(), sizeof(struInputParam.szModuleName) - 1);
	struInputParam.guestID = guestID;

	struInputParam.pRgbdCalibData = calibData.GetBuf();
	struInputParam.nRgbdCalibDataLen = 512;		// TODO

	struInputParam.nTofWidth = nTofWidth;
	struInputParam.nTofHeight = nTofHeight;

	struInputParam.nRgbWidth = nRgbWidth;
	struInputParam.nRgbHeight = nRgbHeight;

	struInputParam.inGrayFormat = inGrayFormat;

	HTOFRGBD hTofRgbd = TOFRGBD_CreateHandle(&struInputParam);
	if (NULL == hTofRgbd)
	{
		printf("TOFRGBD_CreateHandle failed.\n");
		return -1;
	}

	//获取并打印RGB内参
	memset(&struParameters, 0, sizeof(struParameters));
	struParameters.type = TOF_RGBD_PARAM_RgbCameraLensParam;
	retVal = TOFRGBD_GetParameters(hTofRgbd, &struParameters);
	if (TOFRGBDRET_SUCCESS == retVal)
	{
		printf(">> RGB LensParam:\n");
		PrintfLensParameter(&struParameters.uParam.lensParam);
	}
	else
	{
		printf("TOFRGBD_GetParameters(TOF_RGBD_PARAM_RgbCameraLensParam) failed, retVal=0x%08x.\n", retVal);
	}

	*ppstRgbdHandle = hTofRgbd;
	return 0;
}


int TofRgbdSdkUnit(HTOFRGBD pstRgbdHandle)
{
#ifdef RGBD
	if (pstRgbdHandle != NULL)
	{
		TOFRGBD_CloseHandle(pstRgbdHandle);
	}

	return 0;
#endif
}


int TofRgbdProcess(void *pCamHandle, TofRgbdInputData* pDataIn, TofRgbdOutputData* pDataOut)
{
#ifdef RGBD
	camera_handle *pstTofHandle;

	if (!pCamHandle || !pDataIn || !pDataOut)
	{
		printf("[%s] NULL ptr!\n", __func__);
		return -1;
	}

	pstTofHandle = (camera_handle*)pCamHandle;

	const TOFRGBDRET retVal = TOFRGBD_DoCal(pstTofHandle->apstRgbdHandle[0], pDataIn, pDataOut);
	if (TOFRGBDRET_SUCCESS != retVal)
	{
		printf("TOFRGBD_DoCal failed, retVal=0x%08x.\n", retVal);
		return -1;
	}

	return 0;
#endif
}
#endif
