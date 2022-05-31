#include <stdint.h>
#include <sys/types.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/stddef.h>

#include "RockchipRga.h"
#include "RgaUtils.h"


int ConvertNV12ToRGB888(unsigned char *pu8Src, unsigned char *pu8Dst, int width, int height)
{
	int iRet = 0;
	RockchipRga& rkRga(RockchipRga::get());

	if (!pu8Src || !pu8Dst)
	{
		printf("[%s] NULL ptr\n", __func__);
		return -1;
	}

	/********** rga_info_t Init **********/
    rga_info_t rgasrc;
    rga_info_t rgadst;

    memset(&rgasrc, 0, sizeof(rga_info_t));
    rgasrc.fd = -1;
    rgasrc.mmuFlag = 1;
    rgasrc.virAddr = (void*)pu8Src;

    memset(&rgadst, 0, sizeof(rga_info_t));
    rgadst.fd = -1;
    rgadst.mmuFlag = 1;
    rgadst.virAddr = (void*)pu8Dst;

	/************ set the rga_mod ,rotation\composition\scale\copy .... **********/        
	rgasrc.blend = 0xFF0100;

	/********** set the rect_info **********/
    rga_set_rect(&rgasrc.rect, 0, 0, width, height, width, height, RK_FORMAT_YCbCr_420_SP);
    rga_set_rect(&rgadst.rect, 0, 0, width, height, width, height, RK_FORMAT_BGR_888);

	iRet = rkRga.RkRgaBlit(&rgasrc, &rgadst, NULL);

	return iRet;
}

