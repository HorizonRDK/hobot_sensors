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

#ifndef HOBOT_MIPI_CAP_IML_X3PI_HPP_
#define HOBOT_MIPI_CAP_IML_X3PI_HPP_
#include <vector>
#include <string>

#include "hobot_mipi_cap_iml.hpp"

namespace mipi_cam {

class HobotMipiCapIml_x3pi : public HobotMipiCapIml {
 public:
  HobotMipiCapIml_x3pi() {}
  ~HobotMipiCapIml_x3pi() {}

  // 初始化设备环境，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int initEnv(std::string sensor);

  // 复位sensor和时钟，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int reset_sensor(std::string sensor);

  // 判断设备是否支持遍历设备连接的sensor
  // 返回值：true,支持；false，不支持
  bool has_list_sensor();

  // 遍历设备连接的sensor
  std::vector<std::string> list_sensor();
};

}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_X3PI_HPP_
