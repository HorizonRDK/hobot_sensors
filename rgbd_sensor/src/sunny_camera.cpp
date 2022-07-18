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

#include <stdio.h>
#include <signal.h>

#include "camera_control.h"
#ifdef USE_UVC
#include "uvc_control.h"
#include "socam.h"
#endif
// #include "v4l2_capture.h"
#include "tof_depth_process.h"
#include "user_list.h"
#include "camera_hb_cfg.h"

#include "rgbd_node/sunny_camera.h"
/* 1024 > 224*2*2 */
#define UVC_SEND_BUF_SIZE   (1024 + CALIB_RAW_SIZE + POINTCLOUD_SIZE + GREY_DATA_SIZE + 500 * 1024)

typedef struct tagFrameListNode
{
    struct list_head   stList;
    IMAGE_DATA_INFO_S  stImageDataInfo;
    IMAGE_DATA_INFO_S  stEncodeDataInfo;
} FRAME_LIST_NODE_S;

#ifdef USE_UVC
typedef struct UvcSetFilterMsgNode
{
    struct list_head stList;
    UvcTofFilter  stUvcTofFilter;
} UVC_SETFILTER_MSG_NODE_S;
#endif

typedef struct UvcSetFilterMsgListCb
{
    struct list_head    stListHead;
    pthread_mutex_t     stListMutex;
    unsigned int        uiNodeCnt;
} LIST_CB_S;

typedef struct RgbdProcessCb
{
    int isDepthCap;
    int isRgbdCap;
    LIST_CB_S stTofRawListCb;
    LIST_CB_S stRgbYuvListCb;
     LIST_CB_S stRgbListCb;
     LIST_CB_S stUvcSendListCb;
     LIST_CB_S stUvcSetFilterMsgListCb;
    void *pTofHandle;
    void *pRgbHandle;
    void *pUvcHandle;
} RGBD_PROCESS_CB_S;


static int giExit = 0;
static pthread_t gstUvcProcessPid;
static int giIsRuning = 1;
unsigned int guiSendCalibCnt = 0;
RGBD_PROCESS_CB_S gstRgbdPocessCb;
unsigned char au8RgbBuf[RGB_SIZE];
static unsigned char gaucFirmwareVerion[4] = {1, 2, 0, 0};


int Init_ShyCamera()
{
    yuv_to_rgb_table_init();
    return 0;
}
int Uninit_ShyCamera()
{
    return 0;
}
void* Open_ShyCamera(int nCamType)
{
    CAM_PARAM_S stCamParam;
    memset(&stCamParam, 0, sizeof(stCamParam));

    stCamParam.eCamType = (CAM_TYPE_E)nCamType;  // CAM_TYPE_TOF_RGBD;
    void* pCamHdl = OpenCamera(&stCamParam);
    if (!pCamHdl) {
        ROS_printf(0, "[%s]open tof camera failed\n", __func__);
        return NULL;
    }
    // gstRgbdPocessCb.pTofHandle = pTofHandle;
    return pCamHdl;
}
int Start_ShyCamera(void *pCamHdl)
{
    return CameraStreamON(pCamHdl);
}
int Stop_ShyCamera(void *pCamHdl)
{
    return CameraStreamOFF(pCamHdl);
}
int Close_ShyCamera(void **pHdlCam)
{
     if (*pHdlCam) {
        CloseCamera(*pHdlCam);
        *pHdlCam = NULL;
     }
    return 0;
}
int Get_ShyCameraInfo(void *pCamHdl, TCameraParam* pOutInfo)
{
    SomeCalibParam caliParam;
}
/*
int GetVpsFrame_ShyCamera(void *pCamHdl, TShyFrame *pDataOut,int nMain)
{
    int nRet = -1;
    if(pDataOut && pCamHdl) {
        IMAGE_DATA_INFO_S pstImageDataInfo;
        int nRet = GetVpsImageData(pCamHdl,&pstImageDataInfo, nMain);
        pDataOut->height = pstImageDataInfo.height;
        pDataOut->width = pstImageDataInfo.width;
        if (0 == pDataOut->size){
            pDataOut->pucImageData = (unsigned char*)malloc(pstImageDataInfo.uiImageSize);//RGB_YUV_SZIE);
        }
        if (pDataOut->pucImageData)
            memcpy(pDataOut->pucImageData , pstImageDataInfo.pucImageData, pstImageDataInfo.uiImageSize);
        //pDataOut->pucImageData = pstImageDataInfo.pucImageData;
        pDataOut->size = pstImageDataInfo.uiImageSize;
        pDataOut->uiFrameCnt = pstImageDataInfo.uiFrameCnt;
        pDataOut->timeStamp = pstImageDataInfo.timeStamp;
        return nRet;
    }
    return nRet;
}*/
int GetFrame_ShyCamera(void *pCamHdl, TShyFrame *pDataOut)
{
    int nRet = -1;
    if (pDataOut && pCamHdl) {
        IMAGE_DATA_INFO_S pstImageDataInfo;
      	camera_handle *pstCameraHandler = (camera_handle*)pCamHdl;
	    if ((CAM_TYPE_TOF == pstCameraHandler->eCamType) || (CAM_TYPE_TOF_RGBD == pstCameraHandler->eCamType)) {
            pDataOut->height = RAW_HEIGHT;
            pDataOut->width = RAW_WIDTH;
		    pstImageDataInfo.ePixelFormat = PIXEL_FORMAT_RAW;
        } else if (CAM_TYPE_RGB == pstCameraHandler->eCamType) {
            pDataOut->height = RGB_HEIGHT;
            pDataOut->width = RGB_WIDTH;
		    pstImageDataInfo.ePixelFormat = PIXEL_FORMAT_YUV;	
        }

        int nRet = GetImageData(pCamHdl, &pstImageDataInfo);
        if (0 == nRet) {
            if ((CAM_TYPE_TOF == pstCameraHandler->eCamType) || (CAM_TYPE_TOF_RGBD == pstCameraHandler->eCamType)) {
                unsigned short *pu16TofRawData = (unsigned short*)pstImageDataInfo.pucImageData;
                pstImageDataInfo.uiFrameCnt = pu16TofRawData[3] & 0xFFF;
            }
            if (0 == pDataOut->size) {
                pDataOut->pucImageData = (unsigned char*)malloc(pstImageDataInfo.uiImageSize);  // RGB_YUV_SZIE);
            }
            if (pDataOut->pucImageData)
                memcpy(pDataOut->pucImageData , pstImageDataInfo.pucImageData, pstImageDataInfo.uiImageSize);
            // pDataOut->pucImageData = pstImageDataInfo.pucImageData;
            pDataOut->size = pstImageDataInfo.uiImageSize;
            pDataOut->uiFrameCnt = pstImageDataInfo.uiFrameCnt;
            pDataOut->timeStamp = pstImageDataInfo.timeStamp;
            // ROS_printf(2, "[%s]->w:h=%d:%d,sz=%d.\n", __func__, pDataOut->width, pDataOut->height, pDataOut->size);
        }
        return nRet;
    }
    // unsigned short *pu16TofRawData = (unsigned short*)stTmpTofRawInfo.pucImageData;
    // stTmpTofRawInfo.uiFrameCnt = pu16TofRawData[3] & 0xFFF;  // tof
    return nRet;
}
// 获取子码流

int ReleaseFrame_ShyCamera(TShyFrame *pDataOut)
{
    if (pDataOut) {
        pDataOut->width = 0;
        pDataOut->height = 0;
        if (pDataOut->pucImageData) {
            free(pDataOut->pucImageData);
            pDataOut->pucImageData = NULL;
        }
        pDataOut->size = 0;
    }
    return 0;
}
extern uint32_t get_tick_count();
#ifdef CV_CONVERT_CLR
#include <opencv2/opencv.hpp>
#endif
static IMAGE_DATA_INFO_S s_stBgr888DataInfo;
int ReleaseTofResult(TTofRgbResult* pDataOut)
{
    if (pDataOut) {
        if (pDataOut->mOriRes.pU8Graydata)
            delete []pDataOut->mOriRes.pU8Graydata;
    }
    return 0;
}
int Calc_Depth2Tof(void *pCamHdl, TShyFrame *pDataDepthIn, const TShyFrame *pDataYuvIn, TTofRgbResult* pDataOut)
{
    IMAGE_DATA_INFO_S stTofRawDataInfo;
    TOF_DEPTH_DATA_INFO_S stTmpDepthInfo = {0};
    TofRgbdInputData stTofRgbdInput;
    TofRgbdOutputData stTofRgbdOut;
    // s_stBgr888DataInfo.pucImageData = (unsigned char*)malloc(RGB_SIZE);
    stTofRawDataInfo.uiImageSize = pDataDepthIn->size;
    stTofRawDataInfo.pucImageData = pDataDepthIn->pucImageData;
    stTofRawDataInfo.timeStamp = pDataDepthIn->timeStamp;
    stTofRawDataInfo.uiFrameCnt = pDataDepthIn->uiFrameCnt;
    stTofRawDataInfo.ePixelFormat = PIXEL_FORMAT_RAW;

    uint32_t startTm = get_tick_count();
    // ROS_printf(2, "[wuwl-%s]->start dSz=%d,w:h=%d:%d.\n", __func__, pDataDepthIn->size,
    //  pDataDepthIn->width, pDataDepthIn->height);
    int iRet = TofDepthProcessExp(pCamHdl, &stTofRawDataInfo, &stTmpDepthInfo, 0);
    if (0 != iRet) {
        ROS_printf(0, "[wuwl-%s]->err depSz=%d,yuvSz=%d, ret=%d.\n", __func__, pDataDepthIn->size,
         pDataYuvIn->size, iRet);
        return -100;
    }
    uint32_t endTm = get_tick_count();
    // ROS_printf(2, "[wuwl-TofDepthProcess]->laps %d ms ret=%d.\n", endTm - startTm, iRet);
    startTm = endTm;
    UINT32 frameWidth = stTmpDepthInfo.frameWidth;
    UINT32 frameHeight = stTmpDepthInfo.frameHeight;

    pDataOut->mOriRes.timeStamp = stTmpDepthInfo.timeStamp;
    pDataOut->mOriRes.uiFrameCnt = stTmpDepthInfo.uiFrameCnt;
    pDataOut->mOriRes.frameWidth = stTmpDepthInfo.frameWidth;
    pDataOut->mOriRes.frameHeight = stTmpDepthInfo.frameHeight;
    pDataOut->mOriRes.pfPointData = reinterpret_cast<TPointData*>(stTmpDepthInfo.pfPointData);
    pDataOut->mOriRes.grayFormat = (EGRAY_FORMAT)stTmpDepthInfo.grayFormat;
    pDataOut->mOriRes.pGrayData = stTmpDepthInfo.pGrayData;
    // 外部灰度图用的 是 u8 方式，所以，这里转为u8 即可。
    TofGray2U8(stTmpDepthInfo.pGrayData, stTmpDepthInfo.frameWidth, stTmpDepthInfo.frameHeight,
        stTmpDepthInfo.grayFormat, &pDataOut->mOriRes.pU8Graydata);

    pDataOut->mOriRes.pu16GrayData = stTmpDepthInfo.pu16GrayData;
    pDataOut->mOriRes.pu16Confidence = stTmpDepthInfo.pu16Confidence;
    pDataOut->mOriRes.pfNoise = stTmpDepthInfo.pfNoise;
    pDataOut->mOriRes.pDepthData = stTmpDepthInfo.pDepthData;
#ifdef CV_CONVERT_CLR
    cv::Mat imgTmp;
    cv::Mat yuvImg = cv::Mat(RGB_HEIGHT * 3 / 2, RGB_WIDTH, CV_8UC1, pDataYuvIn->pucImageData, 0);
    cv::cvtColor(yuvImg, imgTmp, cv::COLOR_YUV2BGR_NV12);
    memcpy(au8RgbBuf, imgTmp.data, RGB_HEIGHT * 3 * RGB_WIDTH);
#else
    nv12_to_bgr888_buffer(pDataYuvIn->pucImageData, au8RgbBuf, RGB_WIDTH, RGB_HEIGHT);
#endif
    endTm = get_tick_count();
    pDataOut->mOutRgb = au8RgbBuf;
    // ROS_printf(2, "[wuwl-nv12_to_bgr888_buffer]->sz=%d,w:h=%d:%d,laps %d ms ret=%d.\n",
    //  pDataYuvIn->size, pDataYuvIn->width, pDataYuvIn->height, endTm - startTm, iRet);
    startTm = endTm;
    s_stBgr888DataInfo.pucImageData = au8RgbBuf;
    s_stBgr888DataInfo.uiImageSize = RGB_SIZE;
    s_stBgr888DataInfo.ePixelFormat = PIXEL_FORMAT_RGB;
    s_stBgr888DataInfo.uiFrameCnt = pDataYuvIn->uiFrameCnt;
    s_stBgr888DataInfo.timeStamp = pDataYuvIn->timeStamp;

    stTofRgbdInput.pPointCloud = reinterpret_cast<TofRgbdPointCloud*>(stTmpDepthInfo.pfPointData);
    stTofRgbdInput.pGray       = stTmpDepthInfo.pGrayData;
    stTofRgbdInput.nGrayLen    = frameWidth * frameHeight * CaculateGrayPixelBytes(stTmpDepthInfo.grayFormat);
    stTofRgbdInput.pRgb        = s_stBgr888DataInfo.pucImageData;
    stTofRgbdInput.nRgbLen     = s_stBgr888DataInfo.uiImageSize;

    iRet = TofRgbdProcess(pCamHdl, &stTofRgbdInput, &stTofRgbdOut);
    endTm = get_tick_count();
    // ROS_printf(2, "[wuwl-TofRgbdProcess]->laps %d ms,w:h=%d:%d ret=%d.\n", endTm - startTm,
    // frameWidth, frameHeight, iRet);
    pDataOut->mPclRgb.pData = reinterpret_cast<TofRgbdPointClr*>(stTofRgbdOut.colorPointCloud.pData);
    pDataOut->mPclRgb.nWidth = stTofRgbdOut.colorPointCloud.nWidth;
    pDataOut->mPclRgb.nHeight = stTofRgbdOut.colorPointCloud.nHeight;

    return iRet;
}
