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

#ifndef X3_UTILS_H_
#define X3_UTILS_H_


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char sensor_name[10];  // sensor名字
} sensor_id_t;

#define I2C_ADDR_8    1
#define I2C_ADDR_16   2

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))


// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char *res, int max);


#ifdef __cplusplus
};
#endif /* __cplusplus */
#endif // X3_UTILS_H_
