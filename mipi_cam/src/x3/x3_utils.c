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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>

//#include "utils/common_utils.h"
#include "x3_utils.h"
//#include "utils/cJSON.h"

// 获取主芯片类型
E_CHIP_TYPE x3_get_chip_type(void)
{
	int ret = 0;
	FILE *stream;
	char chip_id[16] = {0};

	stream = fopen("/sys/class/socinfo/chip_id", "r");
	if (!stream) {
		ROS_printf("open fail: %s\n", strerror(errno));
		return -1; 
	}   
	ret = fread(chip_id, sizeof(char), 9, stream);
	if (ret != 9) {
		ROS_printf("read fail: %s\n", strerror(errno));
		fclose(stream);
		return -1; 
	}   
	fclose(stream);

	if (strncmp("0xab36300", chip_id, 9) == 0)
		return E_CHIP_X3M; // x3m
	else if (strncmp("0xab37300", chip_id, 9) == 0)
		return E_CHIP_X3E; // x3e
	else return E_CHIP_X3M;
}

typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char sensor_name[10];  // sensor名字
} sensor_id_t;

#define I2C_ADDR_8		1
#define I2C_ADDR_16		2

// mipi sensor的信息数组
sensor_id_t sensor_id_list[] = {
    {2, 0x40, I2C_ADDR_8, 0x0B, "f37"},        // F37
    {1, 0x40, I2C_ADDR_8, 0x0B, "f37"},        // F37
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x29, I2C_ADDR_16, 0x03f0, "gc4663"},  // GC4663
		{2, 0x29, I2C_ADDR_16, 0x03f0, "gc4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
		{2, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
		{2, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
		{2, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
		{1, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {2, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
		{1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
    // {3, 0x36, I2C_ADDR_16, 0x0100, "ov8856"},  // ov8856
    // {3, 0x10, I2C_ADDR_16, 0x0100, "ov8856"},  // ov8856
    // {2, 0x36, I2C_ADDR_16, 0x0100, "os8a10"},  // os8a10
};
#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))

// 获取连接的video_device
char *x3_get_video_device() {
  int i = 0;
  char cmd[128] = {0};
  char result[1024] = {0};
  int length = ARRAY_SIZE(sensor_id_list);
  for (i = 0; i < length; i++) {
    /* 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor
     */
    memset(cmd, '\0', sizeof(cmd));
    memset(result, '\0', sizeof(result));
    if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_8) {
      sprintf(cmd,
              "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
              sensor_id_list[i].i2c_bus,
              sensor_id_list[i].i2c_dev_addr,
              sensor_id_list[i].det_reg);
    } else if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_16) {
      sprintf(cmd,
              "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
              sensor_id_list[i].i2c_bus,
              sensor_id_list[i].i2c_dev_addr,
              sensor_id_list[i].det_reg >> 8,
              sensor_id_list[i].det_reg & 0xFF);
    } else {
      continue;
    }
    exec_cmd_ex(cmd, result, 1024);
    if (strstr(result, "Error") ==
        NULL) {  // 返回结果中不带Error, 说明sensor找到了
      printf("match sensor:%s\n", sensor_id_list[i].sensor_name);
      return sensor_id_list[i].sensor_name;
    }
  }
  return "";
}

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char* res, int max)
{
	if(cmd == NULL || res == NULL || max <= 0)
		return -1;
	FILE *pp = popen(cmd, "r");
	if(!pp)
	{
		ROS_printf("error, cannot popen cmd: %s\n", cmd);
		return -1;
	}
	int length;
	char tmp[1024] = {0};
	length = max;
	if(max > 1024) length = 1024;
	ROS_printf("[%s]->cmd %s ,fp=0x%x,len=%d.\n",__func__,cmd,pp,max);
	while(fgets(tmp, length, pp) != NULL)
	{
		sscanf(tmp, "%s", res);
	}
	pclose(pp);

	return strlen(res);
}

int x3_get_hard_capability(hard_capability_t *capability)
{
	int i = 0;
	char cmd[256];
	char result[1024];

	E_CHIP_TYPE chip_id = x3_get_chip_type();
	ROS_printf("[%s]->chip_type: %s\n",__func__, chip_id == E_CHIP_X3M ? "X3M" : "X3E");
	capability->m_chip_type = chip_id;

	/* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
	HB_MIPI_EnableSensorClock(0);
	HB_MIPI_EnableSensorClock(1);
	HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk
	/*
	for (i = 0; i < ARRAY_SIZE(sensor_id_list); i++) {
		// 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor
		memset(cmd, '\0', sizeof(cmd));
		memset(result, '\0', sizeof(result));
		if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_8) {
			sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr, sensor_id_list[i].det_reg);
		} else if (sensor_id_list[i].i2c_addr_width == I2C_ADDR_16){
			sprintf(cmd, "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1", sensor_id_list[i].i2c_bus,
			sensor_id_list[i].i2c_dev_addr,
			sensor_id_list[i].det_reg >> 8, sensor_id_list[i].det_reg & 0xFF);
		} else {
			continue;
		}
		//i2ctransfer -y -f 3 w2@0x36 0x1 0x0 r1 2>&1 ;这个命令执行会崩溃
		exec_cmd_ex(cmd, result, sizeof(result));
		if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) { // 返回结果中不带Error, 说明sensor找到了
			ROS_printf("--------match sensor:%s\n", sensor_id_list[i].sensor_name);
			capability->m_sensor_list |= sensor_id_list[i].enable_bit;
		}
	}*/

	ROS_printf("support sensor: %d\n", capability->m_sensor_list);
	return 0;
}
