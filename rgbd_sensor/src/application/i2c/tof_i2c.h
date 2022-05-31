#ifndef __I2C_H_
#define __I2C_H_

#include <stdio.h>
#include <linux/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <cstring>
#include <pthread.h>


typedef enum tagI2cFmt {
    I2C_FMT_A8D8, /**< 8 bits Address, 8 bits Data */
    I2C_FMT_A16D8,/**< 16 bits Address 8 bits Data */
    I2C_FMT_A8D16,/**< 8 bits Address 16 bits Data */
    I2C_FMT_A16D16,/**< 16 bits Address 16 bits Data */
    I2C_FMT_END/**< Reserved */
} I2C_FMT_E;

typedef struct tagI2cCb
{
	char acDevPath[32];
	unsigned int uiSlaveAddr;
	int iFd;
	int iIsInit;
} I2C_CB_S;


int I2cCbInit(I2C_CB_S *pstI2cCb, const char* pcI2cDevPath, unsigned int uiSlaveAddr);
void I2cCbUninit(I2C_CB_S* pstI2cCb);
int I2cWrite(I2C_CB_S* pstI2cCb, unsigned short u16Reg, unsigned short u16Value, I2C_FMT_E eFmt);
int I2cRead(I2C_CB_S* pstI2cCb, unsigned short u16Reg, unsigned short *pu16Value, I2C_FMT_E eFmt);
int i2c_block_read_eeprom_a16_d8(I2C_CB_S *pstI2cCb, unsigned short eeprom_addr, unsigned char *buf, unsigned short len);


#endif

