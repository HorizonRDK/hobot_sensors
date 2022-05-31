/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_HB_UTILS_H
#define HB_X2A_VIO_HB_UTILS_H

#include "cJSON.h"
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <semaphore.h>
#include <errno.h>
#include "hb_vio_common.h"
#include "logging.h"

#define HB_VIO_NAME_STR_SIZE (128)
#define HB_VIO_MAX_WEIGHT 3840
#define HB_VIO_MAX_height 2160
#define HB_VIO_MAX_VC_NUM 4

#define XJ3_VIO_DEBUG (1)

#define vio_err(format, ...)			\
do {									\
	char str[30];						\
	struct timespec ts; \
	clock_gettime(CLOCK_MONOTONIC, &ts); \
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_nsec / SECTOMS);	\
	pr_err("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#define vio_warn(format, ...)			\
do {									\
	char str[30];						\
	struct timespec ts; \
	clock_gettime(CLOCK_MONOTONIC, &ts); \
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_nsec / SECTOMS);	\
	pr_warn("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#define vio_log(format, ...)			\
do {									\
	char str[30];						\
	struct timespec ts; \
	clock_gettime(CLOCK_MONOTONIC, &ts); \
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_nsec / SECTOMS);	\
	pr_info("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)

#if XJ3_VIO_DEBUG
#define vio_dbg(format, ...)			\
do {									\
	char str[30];						\
	struct timespec ts; \
	clock_gettime(CLOCK_MONOTONIC, &ts); \
	snprintf(str, sizeof(str), "%ld.%06ld", ts.tv_sec, ts.tv_nsec / SECTOMS);	\
	pr_debug("[%s]%s[%d]: "format"\n", str, __func__, __LINE__, ##__VA_ARGS__);\
} while (0)
#else
#define vio_dbg(format, ...)
#endif

#define XJ3_CPU_SETSIZE 4
#define XJ3_CPU_0 0
#define XJ3_CPU_1 1
#define XJ3_CPU_2 2
#define XJ3_CPU_3 3

#define SIF_YUV_SEPARATE "SIF_YUV_SEPARATE"
#define ISP_YUV_SEPARATE "ISP_YUV_SEPARATE"
#define IPU_YUV_SEPARATE "IPU_YUV_SEPARATE"
#define PYM_YUV_SEPARATE "PYM_YUV_SEPARATE"
#define GDC_YUV_SEPARATE "GDC_YUV_SEPARATE"

#define SIF_IN_BUF_NONCACHED	"SIF_IN_BUF_NONCACHED"
#define SIF_OUT_BUF_NONCACHED	"SIF_OUT_BUF_NONCACHED"
#define ISP_OUT_BUF_NONCACHED	"ISP_OUT_BUF_NONCACHED"
#define IPU_IN_BUF_NONCACHED	"IPU_IN_BUF_NONCACHED"
#define IPU_OUT_BUF_NONCACHED	"IPU_OUT_BUF_NONCACHED"
#define PYM_IN_BUF_NONCACHED	"PYM_IN_BUF_NONCACHED"
#define PYM_OUT_BUF_NONCACHED	"PYM_OUT_BUF_NONCACHED"
#define GDC_IN_BUF_NONCACHED	"GDC_IN_BUF_NONCACHED"
#define GDC_OUT_BUF_NONCACHED	"GDC_OUT_BUF_NONCACHED"

#define SIF_IN_BUF_NOCLEAN	     "SIF_IN_BUF_NOCLEAN"
#define SIF_OUT_BUF_NOINVALID	 "SIF_OUT_BUF_NOINVALID"
#define ISP_OUT_BUF_NOINVALID	 "ISP_OUT_BUF_NOINVALID"
#define IPU_IN_BUF_NOCLEAN	     "IPU_IN_BUF_NOCLEAN"
#define IPU_OUT_BUF_NOINVALID	 "IPU_OUT_BUF_NOINVALID"
#define PYM_IN_BUF_NOCLEAN	     "PYM_IN_BUF_NOCLEAN"
#define PYM_OUT_BUF_NOINVALID	 "PYM_OUT_BUF_NOINVALID"
#define GDC_IN_BUF_NOCLEAN	     "GDC_IN_BUF_NOCLEAN"
#define GDC_OUT_BUF_NOINVALID	 "GDC_OUT_BUF_NOINVALID"

#define SIF_IN_BUF_FLUSHALL_THRESHOLD	"SIF_IN_BUF_FLUSHALL_THRESHOLD"
#define SIF_OUT_BUF_FLUSHALL_THRESHOLD	"SIF_OUT_BUF_FLUSHALL_THRESHOLD"
#define ISP_OUT_BUF_FLUSHALL_THRESHOLD	"ISP_OUT_BUF_FLUSHALL_THRESHOLD"
#define IPU_IN_BUF_FLUSHALL_THRESHOLD	"IPU_IN_BUF_FLUSHALL_THRESHOLD"
#define IPU_OUT_BUF_FLUSHALL_THRESHOLD	"IPU_OUT_BUF_FLUSHALL_THRESHOLD"
#define PYM_IN_BUF_FLUSHALL_THRESHOLD	"PYM_IN_BUF_FLUSHALL_THRESHOLD"
#define PYM_OUT_BUF_FLUSHALL_THRESHOLD	"PYM_OUT_BUF_FLUSHALL_THRESHOLD"
#define GDC_IN_BUF_FLUSHALL_THRESHOLD	"GDC_IN_BUF_FLUSHALL_THRESHOLD"
#define GDC_OUT_BUF_FLUSHALL_THRESHOLD	"GDC_OUT_BUF_FLUSHALL_THRESHOLD"

#define VIO_CACHE_FLUSH_ALL		0
#define VIO_CACHE_FLUSH_ACTUAL_SIZE	0xffffffff
#define VIO_CACHE_FLUSH_NON		0xfffffffe

#define SECTOMS (1000)
#define SECTOUS (1000000)
#define SECTONS (1000000000)
#define MSTONS  (1000000)
#define MAX_SEM_WAIT_TIMEOUT    ((int64_t)(~0ULL>>1))

int set_thread_affinity(pthread_t pid, int cpu_num);
int get_thread_policy(pthread_attr_t *attr);
int set_thread_policy(pthread_attr_t *attr,  int policy);
void get_thread_priority_rang(int policy, int *min, int *max);
int get_thread_priority(pthread_attr_t *attr);
int set_thread_priority(pthread_attr_t *attr, int priority);


void time_add_ms(struct timeval *time, uint16_t ms);
int get_cost_time_ms(struct timeval *start, struct timeval *end);
int dumpToFile(char *filename, char *srcBuf, unsigned int size);
int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     unsigned int size, unsigned int size1);
int dumpToFile2planeStride(char *filename, char *srcBuf, char *srcBuf1,
	unsigned int size, unsigned int size1, int width, int height, int stride);
int32_t timespec_add_ms(struct timespec *ts, int64_t msecs);
int32_t sem_timedwait_msecs(sem_t *sem, int64_t msecs);
int32_t sem_timedwait_relative(sem_t *sem, struct timespec *monotime);
int hb_dump_stat_info(int pipe);
void sem_wait_cond(sem_t *sem);

#endif //HB_X2A_VIO_HB_UTILS_H
