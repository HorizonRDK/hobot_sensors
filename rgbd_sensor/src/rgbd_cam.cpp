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

#define __STDC_CONSTANT_MACROS
#include "rgbd_node/rgbd_cam.hpp"
#include "rgbd_node/video_utils.hpp"

#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace rgbd_node
{

ShyCam::ShyCam(/* args */):m_nCurUseIndex(-1), m_bSyncOutput(1)
{
    for (int nIdx = 0; nIdx < BUF_DEPTH; ++nIdx) {
        m_arrRecvStrmData[nIdx].enDepthStat = m_arrRecvStrmData[nIdx].enClrStat = enDST_EMPTY;
    }
    Init_ShyCamera();
    m_pTofCamHdl = NULL;
    m_pRgbCamHdl = NULL;
    m_nBufNum = BUF_DEPTH * 2;
    memset(&m_arrRecvStrmData, 0 , BUF_DEPTH * 2 * sizeof(TShyStreamData));
}

ShyCam::~ShyCam()
{
    Close_ShyCamera(&m_pTofCamHdl);
    Close_ShyCamera(&m_pRgbCamHdl);
    Uninit_ShyCamera();
}

int ShyCam::InitVideo()
{
    m_pTofCamHdl = Open_ShyCamera(3);   // open rgbd
    m_pRgbCamHdl = Open_ShyCamera(1);  // open rgb
    // TLOG_INFO("Open Device successed .\n");
    m_nDevStat = enDEV_INIT;
    if (!m_pRgbCamHdl || !m_pTofCamHdl) {
        ROS_printf(0, "[%s]->rgb=0x%x,tof=0x%x.\n", __func__, m_pRgbCamHdl, m_pTofCamHdl);
        abort();
    }

  return 0;
}

int ShyCam::GetCurState(int nType)
{
    if (-1 == m_nCurUseIndex)
        return -1;
    if (0 == nType)
        return m_arrRecvStrmData[m_nCurUseIndex].enClrStat;
    return m_arrRecvStrmData[m_nCurUseIndex].enDepthStat;
}

int ShyCam::SetSyncOutput(bool bSync)
{
    if (m_bSyncOutput == bSync)
        return 0;
    usleep(100*1000);

    if (bSync) {
        // 设置同步和对齐 ,设置在拉流之前
        for (int iIdx = 0; iIdx < BUF_DEPTH; iIdx++) {
            m_arrRecvStrmData[iIdx].enDepthStat = m_arrRecvStrmData[iIdx].enClrStat = enDST_EMPTY;
        }
        m_nCurUseIndex = 0;
    }
    m_bSyncOutput = bSync;
    return 0;
}

extern void WriteBinaryFile(char *fileName, void *pData, int nLen);
extern void getLocalTime(char *timeStr, int len, struct timeval tv);

int ShyCam::CalcTofSync(TTofRgbResult *pOutTofRes)
{
    int nRet = -1;
    int nTryNum = 0;
    do {
        if (m_arrRecvStrmData[m_nCurUseIndex].enClrStat >= enDST_READY) {
            // 这里会变成 rgb 数据
            nRet = Calc_Depth2Tof(m_pTofCamHdl, &m_arrRecvStrmData[m_nCurUseIndex].depth_frame,
                &m_arrRecvStrmData[m_nCurUseIndex].clrTof_frame, pOutTofRes);
            break;
        }
        usleep(10*1000);
        ++nTryNum;
    } while (m_nCurUseIndex != -1);

    if (nTryNum > 0) {
        ROS_printf(2, "<====>[%s]->curUse=%d, tryNum=%d, st=d-%d,c-%d.\n", __func__,
            m_nCurUseIndex, nTryNum, (m_nCurUseIndex == -1) ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enDepthStat,
            (m_nCurUseIndex == -1) ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enClrStat);
    }
    return nRet;
}
// 得到可写的索引，取的速度慢，来的快，就完了。。。
static int s_bPrint = 0;
int ShyCam::GetValidUse()
{
    int nIdx = 0;
    uint64_t utTMStamp = 0x7FFFFFFFFFFFFFFF;
    int nRetIdx = -1;
    std::unique_lock<std::mutex> lck_clr(m_mutxUseIdx);
    if (-1 != m_nCurUseIndex)
        return 0;
    for ( ; nIdx < m_nBufNum; ++nIdx) {
        if (s_bPrint) {
            ROS_printf(2, "<==>[%s]->errStamp %lld - %lld Idx=%d-st d:c=%d:%d ,dSz=%d.\n", __func__,
                utTMStamp, m_arrRecvStrmData[nIdx].depth_frame.timeStamp, nIdx,
                m_arrRecvStrmData[nIdx].enDepthStat, m_arrRecvStrmData[nIdx].enClrStat,
                m_arrRecvStrmData[nIdx].depth_frame.size);
        }
        if (enDST_READY == m_arrRecvStrmData[nIdx].enDepthStat &&
            enDST_READY == m_arrRecvStrmData[nIdx].enClrStat) {
            if (utTMStamp > m_arrRecvStrmData[nIdx].depth_frame.timeStamp) {
                utTMStamp = m_arrRecvStrmData[nIdx].depth_frame.timeStamp;
                nRetIdx = nIdx;
            } else {
                ROS_printf(1, "<==>[%s]->errStamp %lld - %lld Idx=%d.\n", __func__,
                    utTMStamp, m_arrRecvStrmData[nIdx].depth_frame.timeStamp, nIdx);
            }
        }
    }
    if (-1 == nRetIdx && s_bPrint) {
        ROS_printf(1, "<==>[%s]->NoBuffer Idx=%d.\n", __func__, nIdx);
        // abort();
    }
    m_nCurUseIndex = nRetIdx;
    return nRetIdx;
}
// 得到可读的索引
int ShyCam::GetValidSave()
{
    int nRetIdx = -1;
    for (int nIdx = 0; nIdx < m_nBufNum; ++nIdx) {
        if (enDST_EMPTY == m_arrRecvStrmData[nIdx].enDepthStat &&
            enDST_EMPTY == m_arrRecvStrmData[nIdx].enClrStat) {
            nRetIdx = nIdx;
            break;
        }
    }
    if (-1 == nRetIdx) {
        //全都满了，那就 把 最大时间戳的用了
        uint64_t utTMStamp = 0;
        int nMaxIdx = -1;
        for (int nIdx = 0 ; nIdx < m_nBufNum; ++nIdx) {
            if (enDST_READY == m_arrRecvStrmData[nIdx].enDepthStat &&
                enDST_READY == m_arrRecvStrmData[nIdx].enClrStat) {
                if (utTMStamp < m_arrRecvStrmData[nIdx].depth_frame.timeStamp) {
                    utTMStamp = m_arrRecvStrmData[nIdx].depth_frame.timeStamp;
                    nMaxIdx = nIdx;
                }
            }
        }
        if (-1 != nMaxIdx) {
            ROS_printf(2, "<==>[GetValidSave]->errBuf %lld-Idx=%d -use=%d,st d-%d,c-%d.\n",
                m_arrRecvStrmData[nMaxIdx].depth_frame.timeStamp, nMaxIdx, m_nCurUseIndex,
                m_arrRecvStrmData[nMaxIdx].enDepthStat , m_arrRecvStrmData[nMaxIdx].enClrStat);
            m_arrRecvStrmData[nMaxIdx].enDepthStat = m_arrRecvStrmData[nMaxIdx].enClrStat = enDST_EMPTY;
            nRetIdx = nMaxIdx;
            s_bPrint = 1;
        }
    }

    return nRetIdx;
}
// 参考：通过深度相机获取一帧图像
//  image 输出的图像，接着将image输入到pcl里面
bool ShyCam::GetDepthFrame(TShyFrame &imgDepth, int nCheck)
{
    if (!nCheck)
    {
        if (m_nCurUseIndex != -1) {
            imgDepth = m_arrRecvStrmData[m_nCurUseIndex].depth_frame;
            return true;
        }
        return false;
    } else {
        GetValidUse();
        if (m_nCurUseIndex != -1) {
            if (enDST_READY == m_arrRecvStrmData[m_nCurUseIndex].enDepthStat) {
                m_arrRecvStrmData[m_nCurUseIndex].enDepthStat = enDST_USING;
                imgDepth = m_arrRecvStrmData[m_nCurUseIndex].depth_frame;
                // ROS_printf("<==>[GetDepthFrame]->idx=%d, tm=%lld.\n", m_nCurUseIndex,
                // m_arrRecvStrmData[m_nCurUseIndex].depth_frame.timeStamp);
                return true;
            }
        } else {
            usleep(5 * 1000);
        }
        return false;
    }
}

bool ShyCam::ReleaseDepthFrame()
{
    m_arrRecvStrmData[m_nCurUseIndex].enDepthStat = enDST_USED;
    release2save();
    return true;
}

bool ShyCam::GetClrFrame(TShyFrame &imgClr, int nCheck)
{
    if (!nCheck) {
        if (m_nCurUseIndex != -1) {
            imgClr = m_arrRecvStrmData[m_nCurUseIndex].clrTof_frame;  // clr_frame;
            return true;
        }
        return false;
    } else {
        GetValidUse();
        if (m_nCurUseIndex != -1) {
            if (enDST_READY == m_arrRecvStrmData[m_nCurUseIndex].enClrStat) {
                m_arrRecvStrmData[m_nCurUseIndex].enClrStat = enDST_USING;
                imgClr = m_arrRecvStrmData[m_nCurUseIndex].clrTof_frame;  // clr_frame;
                // ROS_printf(2, "<==>[ClrFrame]->get Idx=%d,tm=[%lld], fcc=%d w:h=%d:%d.\n",
                //  m_nCurUseIndex, imgClr.timeStamp, imgClr.fcc, imgClr.width, imgClr.height);
                return true;
            }
        } else {
            usleep(5*1000);
        }
        return false;
    }
    return true;
}

bool ShyCam::ReleaseClrFrame()
{
    m_arrRecvStrmData[m_nCurUseIndex].enClrStat = enDST_USED;
    release2save();
    return true;
}
bool ShyCam::ReleasePair()
{
    if (m_arrRecvStrmData[m_nCurUseIndex].enDepthStat == enDST_USED &&
        m_arrRecvStrmData[m_nCurUseIndex].enClrStat == enDST_USED) {
        std::unique_lock<std::mutex> lck_clr(m_mutxUseIdx);
        m_arrRecvStrmData[m_nCurUseIndex].enDepthStat = enDST_EMPTY;
        m_arrRecvStrmData[m_nCurUseIndex].enClrStat = enDST_EMPTY;

        // ROS_printf(2, "<=****=>[%s]->End UseIdx=%d to -1.\n", __func__, m_nCurUseIndex);
        m_nCurUseIndex = -1;
        return true;
    }
    return false;
}
void ShyCam::release2save()
{
    // 切换保存的区域
}
static int nTestNum = 0;
// 获取深度图，yuv 图
int ShyCam::doCapStreamLoop()
{
    // prctl(PR_SET_NAME, "Stream");
    int nTryNum = 0;
    m_lStartTM = GetTickCount();
    m_nPrintNum = m_nDepthNum = 0;
    m_nPrintNum = 100;
    m_nPrintPairNum = m_nPairNum = 0;
    ROS_printf(2, "<========>[%s]->Start.\n", __func__);
    while (!m_bThrdQuit) {
        if (m_bSyncOutput && enDEV_START == m_nDevStat) {
            int nNewPair = 1;  // 新组
            int nCurSave = GetValidSave();  // 正在保存的索引
            uint32_t msStart = 0, msEnd = 0;
            msStart = GetTickCount();
            if (-1 == nCurSave) {
                usleep(10*1000);
                continue;
            }
            do {
                // 先取深度图，以 深度图为基准，取了深度图，就再取 yuv 图。
                nNewPair = 0;
                nTryNum = 0;
                // ROS_printf(2, "=>startget:%d ,use(%d)st=d-%d:c-%d.\n", nCurSave, m_nCurUseIndex,
                //  m_arrRecvStrmData[nCurSave].enDepthStat,
                //  m_arrRecvStrmData[nCurSave].enClrStat);
                if (m_arrRecvStrmData[nCurSave].enDepthStat == enDST_EMPTY) {
                    // Depth
                    if (0 == GetFrame_ShyCamera(m_pTofCamHdl, &m_arrRecvStrmData[nCurSave].depth_frame)) {
                        m_nDepthNum++;
                        m_arrRecvStrmData[nCurSave].enDepthStat = enDST_READY;
                        if (++m_nPrintNum > 100) {
                            m_nPrintNum = 0;
                            msEnd = GetTickCount();
                            ROS_printf(2, "--->[Depth: fps=%d, Save:use=%d:%d, st rgb:dep=%d:%d, W:H=%03d:%03d .\n",
                                m_nDepthNum * 1000 / (msEnd - m_lStartTM),
                                nCurSave, m_nCurUseIndex,
                                m_nCurUseIndex == -1 ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enClrStat,
                                m_nCurUseIndex == -1 ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enDepthStat,
                                m_arrRecvStrmData[nCurSave].depth_frame.width,
                                m_arrRecvStrmData[nCurSave].depth_frame.height);
                        }
                    } else {
                        continue;
                    }
                } else {
                    // if (0 == GetVpsFrame_ShyCamera(m_pRgbCamHdl, &m_arrRecvStrmData[nCurSave].clrTof_frame, 0) &&
                    //  0 == GetVpsFrame_ShyCamera(m_pRgbCamHdl, &m_arrRecvStrmData[nCurSave].clr_frame, 1)) {
                    if (0 == GetFrame_ShyCamera(m_pRgbCamHdl, &m_arrRecvStrmData[nCurSave].clrTof_frame)) {
                        m_arrRecvStrmData[nCurSave].enClrStat = enDST_READY;
                    } else {
                        usleep(5*1000);
                        continue;
                    }
                }
                if (m_arrRecvStrmData[nCurSave].enClrStat >= enDST_READY &&
                    m_arrRecvStrmData[nCurSave].enDepthStat >= enDST_READY) {
                    nNewPair = 1;
                    ++m_nPairNum;
                    if (++m_nPrintPairNum > 10) {
                        msEnd = GetTickCount();
                        m_nPrintPairNum = 0;
                        ROS_printf(2, "--->[Pair]->fps=%d-%d, save:use=%d:%d, st rgb:dep=%d:%d.\n",
                            m_nPairNum * 1000 / (msEnd - m_lStartTM), m_nPairNum, nCurSave, m_nCurUseIndex,
                            m_nCurUseIndex == -1 ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enClrStat,
                            m_nCurUseIndex == -1 ? -1 : m_arrRecvStrmData[m_nCurUseIndex].enDepthStat);
                    }
                    // TLOG_INFO("End Recv Frame :%d ,use=%d, tmLaps=%d.\n", nCurSave, m_nCurUseIndex, msEnd - msStart);
                    break;
                }
            } while (1);
        }
    }
}

int ShyCam::childStart()
{
    if (m_pTofCamHdl && m_pRgbCamHdl) {
        int nRet = Start_ShyCamera(m_pTofCamHdl);
        nRet = Start_ShyCamera(m_pRgbCamHdl);
        ROS_printf(2, "[%s]-> ret=%d !\n", __func__, nRet);
        return nRet;
    }
    return -1;
}

int ShyCam::childStop()
{
    Stop_ShyCamera(m_pTofCamHdl);
    Stop_ShyCamera(m_pRgbCamHdl);
    return 0;
}
}  // namespace rgbd_node
