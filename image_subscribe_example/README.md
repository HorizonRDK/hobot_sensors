Getting Started with image_subscribe_example
=======

# Intro

image_subscribe_example package用于显示接收 ROS2 Node 发布的image msg。支持ROS标准格式，也支持 share mem 方式订阅。

# Build
---
## Dependency

ros package：
- sensor_msgs
- hbm_img_msgs

其中cv_bridge为ROS开源的package，需要手动安装，具体安装方法：

```cpp
# 方法1，直接使用apt安装，以cv_bridge安装举例
sudo apt-get install ros-foxy-cv-bridge -y

# 方法2，使用rosdep检查并自动安装pkg编译的依赖项
# 安装ros pkg依赖下载⼯具rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# 在ros的⼯程路径下执⾏安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖
rosdep install -i --from-path . --rosdistro foxy -y
```

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。
### 编译选项

SHARED_MEM

- shared mem（共享内存传输）使能开关，默认关闭（OFF），编译时使用-DSHARED_MEM=ON命令打开。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- 对于shared mem通信方式，当前不支持订阅 compressed topic。
- CMakeLists.txt中指定Mipi_cam package的安装路径，默认为`../install/image_subscribe_example`。

### X3 Ubuntu系统上编译
1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已依赖pkg ，详见 Dependency 部分

2、编译：
  - 订阅share mem 方式发布的图片：`colcon build --packages-select image_subscribe_example --cmake-args -DSHARED_MEM=ON`
  - 支持订阅ROS2标准格式图片：`colcon build --packages-select image_subscribe_example`或`colcon build --packages-select image_subscribe_example --cmake-args -DSHARED_MEM=OFF`。

### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明：http://gitlab.hobot.cc/robot_dev_platform/robot_dev_config/blob/dev/README.md
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令： 

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select image_subscribe_example \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
     -DSYS_ROOT=/mnt/test/cc_ws/sysroot_docker \
     -DSHARED_MEM=ON
  ```
- 其中SYS_ROOT为交叉编译系统依赖路径，此路径具体地址详见第1步“编译环境确认”的交叉编译说明。

# Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run image_subscribe_example subscribe_example
#可以输入参数进行订阅并保存图片：默认是不保存图片，除非设置了save_dir参数
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed -p save_dir:=/userdata

指明topic 为 hbmem_img，接收 发布端通过share mem pub 的数据：
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=hbmem_img
```
