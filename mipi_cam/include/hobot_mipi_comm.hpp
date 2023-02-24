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

#ifndef MIPI_MIPI_COMM_HPP_
#define MIPI_MIPI_COMM_HPP_

#include <string>

extern "C" {
#include <string.h>
#include <stdlib.h>
}

namespace mipi_cam
{

struct NodePara {
  std::string camera_name_;
  std::string video_device_name_;
  int channel_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;
  std::string out_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;
};

}  // namespace mipi_cam
#endif  // MIPI_MIPI_COMM_HPP_
