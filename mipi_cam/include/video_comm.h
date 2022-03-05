#ifndef VIDEO_COMM_H
#define VIDEO_COMM_H
 extern "C" {
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/time.h>
    
#include <linux/fb.h>
extern int ROS_printf(char *fmt, ...);
}

#include <pthread.h>
struct TCapFrame;

typedef void (*CapFrame_Callback_t)(struct TCapFrame *frame, void *user_args);
//摄像头帧数据基本信息
struct TCapFrame {
	struct timeval timestamp;
	int fcc;
	void *mem;
	int width;
	int height;
	int length;
};
//usb 摄像头基本配置
typedef struct _Cam_Info_t{
	char devName[24];
	int width;
	int height;
	int fps;
	int fcc;	//mpg,yuv,h264 ...
}TCamInfo;

class CVideoComm
{
private:
	/* data */
public:
	CVideoComm(/* args */):m_nDevStat(0){};
	~CVideoComm(){};
	virtual int StartStream(CapFrame_Callback_t pFrameCB,void *pArg){
		if (0!=m_nDevStat && pFrameCB && 0==childStart()) {
			int r;
			m_cbCapFrame = pFrameCB;
			m_pOwner = pArg;
			m_bThrdQuit = 0;
			r = pthread_create(&m_pidCam, NULL,
					CapStreaming_loop, this);
			if (r < 0) {
				ROS_printf("[%s]->pthread create failed\n", __func__);
				return -100;
			}
			ROS_printf("[%s]->pthread create sucess\n", __func__);
			return 0;
		}
		return -1;
	};
	virtual int StopStream(){
		/* if has user callback, needs to join the streaming_loop thread */
		if (m_cbCapFrame && m_pidCam) {
			m_bThrdQuit = 1;
		    int r = pthread_join(m_pidCam, NULL);
			if (r < 0) {
				ROS_printf("streaming_loop pthread join failed\n");
				return -100;
			}
			return childStop();
		}
		return -1;
	}
protected:
	TCamInfo m_oCamInfo;
	char m_bThrdQuit;
	int m_nDevStat;	//0 表示未准备好，1 表示 准备好
	pthread_t m_pidCam;
	CapFrame_Callback_t m_cbCapFrame;
	void *m_pOwner;
	static void *CapStreaming_loop(void *arg){
		CVideoComm *cam = (CVideoComm *)arg;
		if(cam)
			cam->doCapStreamLoop();
		return (void*)0;
	}
	virtual int doCapStreamLoop()=0;
	virtual int childStart()=0;
	virtual int childStop()=0;
};

#endif // VIDEO_CAP_H

