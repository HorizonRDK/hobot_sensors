Getting Started with hbm_img_msgs
=======


# Intro

hbm_img_msgs package用于 hbmem传输1080P RGB24位 以下分辨率的图片数据 定义的消息结构。NV12 也可以进行传输，读取数据的时候，根据 encoding 和 data_size 去读取。

# Build
colcon build --packages-select hbm_img_msgs

## Dependency


## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

- CMakeLists.txt中指定Mipi_cam package的安装路径，默认为`../install/hbm_img_msgs`。
- 编译：`colcon build --packages-select hbm_img_msgs`

# Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
// 头文件路径
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
```
