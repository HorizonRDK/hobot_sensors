// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __SUNNY_CAMERA_H__
#define __SUNNY_CAMERA_H__

typedef struct _tag_shyFrame {
    // unsigned long long timeStamp;
    int64_t timeStamp;
    int fcc;
    unsigned char *pucImageData;
    int width;
    int height;
    int size;
    unsigned int uiFrameCnt;
}TShyFrame;

// 灰度数据格式
typedef enum tag_GRAY_FORMAT
{
    EGRAY_FORMAT_UINT8  = 0,  // 8位数据
    EGRAY_FORMAT_UINT16,  // 无符号16位数据
    EGRAY_FORMAT_FLOAT,  // 浮点型数据
    EGRAY_FORMAT_BGRD,  // 每像素32位， 按B/G/R/D顺序存放
}EGRAY_FORMAT;

typedef struct tag_PointData
{
    float x;
    float y;
    float z;
}TPointData;

// TOF模组内参和畸变（通用模型）
typedef struct tagTofMdlLensGeneral
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
}TTofMdlLensGeneral;

// TOF模组内参和畸变（鱼眼模型）
typedef struct tagTofMdlLensFishEye
{
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float k4;
}TTofMdlLensFishEye;
typedef struct tag_CamInfo
{
    unsigned int nIdx;  // 1---general有效, 2---fishEye有效

    union
    {
        // [第1种]: 普通模型
        TTofMdlLensGeneral general;  // 普通模型

        // [第2种]: 鱼眼模型
        TTofMdlLensFishEye fishEye;  // 鱼眼模型
    }uParam;
}TCameraParam;

typedef struct tof_depth_data
{
    // unsigned long long  timeStamp;  // 时间戳
    int64_t timeStamp;
    unsigned int  uiFrameCnt;  // 帧计数
    unsigned int  frameWidth;  // 深度图宽度
    unsigned int  frameHeight;  // 深度图高度
    TPointData *pfPointData;  // 点云数据
    EGRAY_FORMAT grayFormat;  // pGrayData内数据格式
    void   *pGrayData;  // 灰度数据
    void   *pU8Graydata;  // u8 灰度图 ,wuwl
    // unsigned short *pu16GrayData;  // u16灰度数据
    uint16_t *pu16GrayData;
    // unsigned short *pu16Confidence;  // 置信度
    uint16_t *pu16Confidence;
    float *pfNoise;  // 噪声
    float* pDepthData;  // 射线距离
} TofDepth_Info;

// 彩色点云
typedef struct tagTofRgbdPointClr
{
    float x;
    float y;
    float z;

    float r;
    float g;
    float b;
}TofRgbdPointClr;

// 彩色点云图像信息
typedef struct tagTofRgbd_PCDColor
{
    TofRgbdPointClr* pData;  // 数据总长：nWidth * nHeight * sizeof(pData[0])
    unsigned int            nWidth;
    unsigned int            nHeight;
}TTofRgb_PCDClr;
typedef struct tagTofRgbd_Res
{
    void * mOutRgb;
    TofDepth_Info mOriRes;
    TTofRgb_PCDClr mPclRgb;
}TTofRgbResult;

#ifdef __cplusplus
extern "C" {
#endif

extern int ROS_printf(int nLev, char *fmt, ...);
// 取流
// 回调函数
int Init_ShyCamera();
int Uninit_ShyCamera();
void* Open_ShyCamera(int nCamType);
int Start_ShyCamera(void *pCamHdl);
int Stop_ShyCamera(void *pCamHdl);
int Close_ShyCamera(void **pCamHdl);
// int GetVpsFrame_ShyCamera(void *pCamHdl, TShyFrame *pDataOut,int nMain);

int GetFrame_ShyCamera(void *pCamHdl, TShyFrame *pDataOut);
int ReleaseFrame_ShyCamera(TShyFrame *pDataOut);
int Get_ShyCameraInfo(void *pCamHdl, TCameraParam* pOutInfo);
// TOF 计算接口
int Calc_Depth2Tof(void *pCamHdl, TShyFrame *pDataDepthIn, const TShyFrame *pDataRgbIn, TTofRgbResult* pDataOut);
int ReleaseTofResult(TTofRgbResult* pDataOut);
// 计算融合

#ifdef __cplusplus
};
#endif

#endif
