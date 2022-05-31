#include <sys/ioctl.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>
#include <arm_neon.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "tof_i2c.h"


int I2cCbInit(I2C_CB_S *pstI2cCb, const char* pcI2cDevPath, unsigned int uiSlaveAddr)
{
	if (!pstI2cCb || !pcI2cDevPath)
	{
		printf("%s NULL prt!\n", __func__);
		return -1;
	}

	memset(pstI2cCb, 0, sizeof(I2C_CB_S));

	snprintf(pstI2cCb->acDevPath, 32, "%s", pcI2cDevPath);
	pstI2cCb->uiSlaveAddr = uiSlaveAddr;

	pstI2cCb->iFd = open(pstI2cCb->acDevPath, O_RDWR);
	if (pstI2cCb->iFd < 0)
	{
		printf("open %s failed\n", pstI2cCb->acDevPath);
		return -1;
	}

	pstI2cCb->iIsInit = 1;

	return 0;
}

void I2cCbUninit(I2C_CB_S* pstI2cCb)
{
	if (!pstI2cCb)
	{
		printf("pstI2cCb is NULL!\n");
		return;
	}
	
	close(pstI2cCb->iFd);
	pstI2cCb->iFd = -1;
	pstI2cCb->iIsInit = 0;
}


int I2cWrite(I2C_CB_S* pstI2cCb, unsigned short u16Reg, unsigned short u16Value, I2C_FMT_E eFmt)
{
	int iRet = 0;
	int iFd;
	unsigned int uiSlaveAddr;
	unsigned char data[4];

	if (!pstI2cCb)
	{
		printf("%s pstI2cCb is NULL!\n", __func__);
		return -1;
	}

	if (pstI2cCb->iIsInit != 1)
	{
		printf("%s I2cCb have not init\n", __func__);
		return -1;
	}

	iFd = pstI2cCb->iFd;
	uiSlaveAddr = pstI2cCb->uiSlaveAddr;

	iRet = ioctl(iFd, I2C_TENBIT, 0);   //7bit slave address
	iRet = ioctl(iFd, I2C_SLAVE_FORCE, uiSlaveAddr);
	if(iRet < 0)
	{
		printf("%s ioctl error[0x%x]\n", __func__, iRet);
		return -1;
	}

	memset(data, 0, sizeof(data));
	
	switch(eFmt) 
	{
		case I2C_FMT_A8D8:
			//printf("[%s]: I2C_FMT_A8D8, reg = %#x, value = %#x \n", __func__, reg, value);
			data[0] = u16Reg & 0xff;
			data[1] = u16Value & 0xff;
			if (write(iFd, data, 2) != 2) 
			{
				perror("Write Register");
				return -1;
			}
			break;
		case I2C_FMT_A16D8:
			//printf("[%s]: I2C_FMT_A16D8, reg = %#x, value = %#x \n", __func__, reg, value);
			data[0] = (u16Reg >> 8) & 0xff;
			data[1] = u16Reg & 0xff;
			data[2] = u16Value & 0xff;
			if (write(iFd, data, 3) != 3) {
				perror("Write Register");
				return -1;
			}
			break;
		case I2C_FMT_A8D16:
			//printf("[%s]: I2C_FMT_A8D16, reg = %#x, value = %#x \n", __func__, reg, value);
			data[0] = u16Reg & 0xff;
			data[1] = (u16Value >> 8) & 0xff;
			data[2] = (u16Value ) & 0xff;
			if (write(iFd, data, 3) != 3) {
				perror("Write Register");
				return -1;
			}
			break;
		case I2C_FMT_A16D16:
			//printf("[%s]: I2C_FMT_A16D16, reg = %#x, value = %#x \n", __func__, reg, value);
			data[0] = (u16Reg >> 8) & 0xff;
			data[1] = (u16Reg ) & 0xff;
			data[2] = (u16Value >> 8) & 0xff;
			data[3] = (u16Value ) & 0xff;
			if (write(iFd, data, 4) != 4) {
				perror("SetRegisterPair");
				return -1;
			}
			break;

		default:
			printf("%s Error I2C format\n", __func__);
			return -1;
	}

	return 0;
}


int I2cRead(I2C_CB_S* pstI2cCb, unsigned short u16Reg, unsigned short *pu16Value, I2C_FMT_E eFmt)
{
	int iRet = 0;
	int iFd;
	unsigned int uiSlaveAddr;
	unsigned char reg_addr[2];
	unsigned char buf[2];

	if (!pstI2cCb)
	{
		printf("%s pstI2cCb is NULL!\n", __func__);
		return -1;
	}

	if (pstI2cCb->iIsInit != 1)
	{
		printf("%s I2cCb have not init\n", __func__);
		return -1;
	}

	iFd = pstI2cCb->iFd;
	uiSlaveAddr = pstI2cCb->uiSlaveAddr;

	iRet = ioctl(iFd, I2C_TENBIT, 0);   //7bit slave address
	iRet = ioctl(iFd, I2C_SLAVE_FORCE, uiSlaveAddr);
	if(iRet < 0)
	{
		printf("%s ioctl error[0x%x]\n", __func__, iRet);
		return -1;
	}

	memset(reg_addr, 0, sizeof(reg_addr));
	
	switch(eFmt) 
	{
		case I2C_FMT_A8D8:
			//printf("[%s]: I2C_FMT_A8D8 \n", __func__);
			reg_addr[0] =  u16Reg & 0xff;
			if (write(iFd, reg_addr, 1) != 1) {
				perror("Read RegisterPair set register");
				return -1;
			}
			if (read(iFd, pu16Value, 1) != 1) {
				perror("Read RegisterPair read value");
				return -1;
			}
			//printf("[%s]: read val[0] = %#x \n", __func__, val[0]);

			break;
		case I2C_FMT_A16D8:
			//printf("[%s]: I2C_FMT_A16D8 \n", __func__);
			reg_addr[0] = (u16Reg >> 8) & 0xff;
			reg_addr[1] =  u16Reg & 0xff;
			//printf("reg_addr[0]: %#x\n", reg_addr[0]);
			//printf("reg_addr[1]: %#x\n", reg_addr[1]);
			if (write(iFd, reg_addr, 2) != 2) {
				perror("Read RegisterPair set register");
				return -1;
			}
			if (read(iFd, pu16Value, 1) != 1) {
				perror("Read RegisterPair read value");
				return -1;
			}
			//printf("[%s]: read val[0] = %#x \n", __func__, val[0]);
			break;
		case I2C_FMT_A8D16:
			//printf("[%s]: I2C_FMT_A8D16 \n", __func__);
			reg_addr[0] =  u16Reg & 0xff;
			//printf("reg_addr[0]: %#x\n", reg_addr[0]);
			if (write(iFd, reg_addr, 1) != 1) {
				perror("Read RegisterPair set register");
				return -1;
			}
			if (read(iFd, pu16Value, 2) != 2) {
				perror("Read RegisterPair read value");
				return -1;
			}

			*pu16Value = (unsigned short)(((unsigned short)(buf[0]<<8)) | ((unsigned short)(buf[1])));///buf0是高8bit
			//printf("[%s]: read val[0] = %#x \n", __func__, val[0]);

			break;
		case I2C_FMT_A16D16:
			//printf("[%s]: I2C_FMT_A16D16 \n", __func__);
			reg_addr[0] = (u16Reg >> 8) & 0xff;
			reg_addr[1] =  u16Reg & 0xff;
			if (write(iFd, reg_addr, 2) != 2) {
				perror("Read RegisterPair set register");
				return -1;
			}
			if (read(iFd, buf, 2) != 2) {
				perror("Read RegisterPair read value");
				return -1;
			}

			*pu16Value = (unsigned short)(((unsigned short)(buf[0]<<8)) | ((unsigned short)(buf[1])));///buf0是高8bit
			//printf("[%s]: read val[0] = %#x \n", __func__, val[0]);
			break;

		default:
			printf("%s Error I2C format\n", __func__);
			return -1;
	}

	return 0;
}

int i2c_block_read_eeprom_a16_d8(I2C_CB_S *pstI2cCb, unsigned short eeprom_addr, unsigned char *buf, unsigned short len)
{                                                                                                                                                                                            
	struct i2c_rdwr_ioctl_data msg_rdwr;
	struct i2c_msg             msg[2];
	int ret = 0;
	int fd;
	unsigned char dev_addr;
	unsigned char reg[2];
	
	if (0 == pstI2cCb->iIsInit)
	{
		printf("I2cCb have not init\n");
		return -1;
	}

	fd = pstI2cCb->iFd;
	dev_addr = pstI2cCb->uiSlaveAddr;

	ioctl(fd,I2C_TIMEOUT,2);//超时时间
	ioctl(fd,I2C_RETRIES,1);//重复次数

	reg[0] = eeprom_addr >> 8;
	reg[1] = eeprom_addr & 0xff;

	msg[0].addr = dev_addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg;


	msg[1].addr = dev_addr;
	msg[1].flags = 1;
	msg[1].len = len;
	msg[1].buf = buf;

	msg_rdwr.msgs = msg;
	msg_rdwr.nmsgs = 2;

	ret = ioctl(fd, I2C_RDWR, &msg_rdwr);
	if (ret < 0) {
		printf("\n %s I2C_RDWR dev_addr 0x%x read 0x%x error, return %d\n", __func__, dev_addr, eeprom_addr, ret);
		return ret;
	}
	
	return 0;
}

