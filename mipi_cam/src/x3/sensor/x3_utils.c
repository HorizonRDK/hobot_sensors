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
#include "x3_utils.h"

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char* res, int max) {
  if (cmd == NULL || res == NULL || max <= 0)
    return -1;
  FILE *pp = popen(cmd, "r");
  if (!pp) {
    ROS_printf("error, cannot popen cmd: %s\n", cmd);
    return -1;
  }
  int length;
  char tmp[1024] = {0};
  length = max;
  if (max > 1024)
    length = 1024;
  ROS_printf("[%s]->cmd %s, fp=0x%x, len=%d.\n", __func__, cmd, pp, max);
  while (fgets(tmp, length, pp) != NULL) {
    sscanf(tmp, "%s", res);
  }
  pclose(pp);
  return strlen(res);
}
