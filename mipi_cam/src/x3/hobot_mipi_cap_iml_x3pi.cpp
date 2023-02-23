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

#include "hobot_mipi_cap_iml_x3pi.hpp"

#include "x3_utils.h"

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>

namespace mipi_cam {

int HobotMipiCapIml_x3pi::initEnv(std::string sensor) {
  std::vector<std::string> sys_cmds = {
    "echo 19 > /sys/class/gpio/export",
    "echo out > /sys/class/gpio/gpio19/direction",
    "echo 0 > /sys/class/gpio/gpio19/value",
    "sleep 0.2",
    "echo 1 > /sys/class/gpio/gpio19/value",
    "echo 19 > /sys/class/gpio/unexport",
    "echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en",
    "echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en",
    "echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq",
    "echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq",
    "echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart",
    "echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart"
  };
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  return 0;
}

// mipi sensor的信息数组
sensor_id_t sensor_id_list_x3pi[] = {
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
};

std::vector<std::string> HobotMipiCapIml_x3pi::list_sensor() {
  std::vector<std::string> device;
  int i = 0;
  char cmd[256];
  char result[1024];
  // #define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))

  /* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
  HB_MIPI_EnableSensorClock(0);
  HB_MIPI_EnableSensorClock(1);
  // HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk
  for (i = 0; i < ARRAY_SIZE(sensor_id_list_x3pi); i++) {
    // 通过i2ctransfer命令读取特定寄存器，判断是否读取成功来判断是否支持相应的sensor
    memset(cmd, '\0', sizeof(cmd));
    memset(result, '\0', sizeof(result));
    if (sensor_id_list_x3pi[i].i2c_addr_width == I2C_ADDR_8) {
      sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
              sensor_id_list_x3pi[i].i2c_bus,
              sensor_id_list_x3pi[i].i2c_dev_addr,
              sensor_id_list_x3pi[i].det_reg);
    } else if (sensor_id_list_x3pi[i].i2c_addr_width == I2C_ADDR_16) {
      sprintf(cmd,
              "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
              sensor_id_list_x3pi[i].i2c_bus,
              sensor_id_list_x3pi[i].i2c_dev_addr,
              sensor_id_list_x3pi[i].det_reg >> 8,
              sensor_id_list_x3pi[i].det_reg & 0xFF);
    } else {
      continue;
    }
    // i2ctransfer -y -f 3 w2@0x36 0x1 0x0 r1 2>&1 ;这个命令执行会崩溃
    exec_cmd_ex(cmd, result, sizeof(result));
    if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) {
      // 返回结果中不带Error, 说明sensor找到了
      device.push_back(sensor_id_list_x3pi[i].sensor_name);
    }
  }
  return device;
}

bool HobotMipiCapIml_x3pi::has_list_sensor() {
  return true;
}

int HobotMipiCapIml_x3pi::reset_sensor(std::string sensor) {
  std::cout << "HobotMipiCapIml_x3pi::reset_sensor" << std::endl;
  int mipi_idx = vin_info_.snsinfo.sensorInfo.entry_index;
  // x3pi两个sensor使用的同一个reset管脚，只需要复位一次
  (void)system("echo 19 > /sys/class/gpio/export");
  (void)system("echo out > /sys/class/gpio/gpio19/direction");
  (void)system("echo 0 > /sys/class/gpio/gpio19/value");
  (void)system("sleep 0.2");
  (void)system("echo 1 > /sys/class/gpio/gpio19/value");
  (void)system("echo 19 > /sys/class/gpio/unexport");
  (void)system("echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en");
  (void)system("echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq");
  (void)system("echo 1 > /sys/class/vps/mipi_host2/param/stop_check_instart");
}

int HobotMipiCapIml_x3pi::get_sensor_bus(std::string &sensor_name) {
  int ret = 0xff;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    ret = 1;
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
     ret = 1;
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
     ret = 1;
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
     ret = 1;
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
     ret = 1;
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
     ret = 1;
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
     ret = 1;
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
     ret = 1;
  }
  return ret;
}

}  // namespace mipi_cam
