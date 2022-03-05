/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef X3_VIO_VDEC_H_
#define X3_VIO_VDEC_H_

#ifdef SW_VDEC
#include "vio/hb_comm_vdec.h"
#include "vio/hb_vdec.h"

#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libavutil/avutil.h"

#include "x3_sdk_wrap.h"

int x3_vdec_init(VDEC_CHN vdecChn, VDEC_CHN_ATTR_S* vdecChnAttr);
int x3_vdec_start(VDEC_CHN vdecChn);
int x3_vdec_stop(VDEC_CHN vdecChn);
int x3_vdec_deinit(VDEC_CHN vdecChn);

int AV_open_stream_file(char *FileName, AVFormatContext **avContext,
		AVPacket *avpacket);
int AV_read_frame(AVFormatContext *avContext, AVPacket *avpacket,
		av_param_t *av_param, vp_param_t *vp_param);
#endif
#endif // X3_VIO_VDEC_H_

