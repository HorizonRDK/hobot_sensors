# Getting Started with Mipi_Cam Node
---
# Intro
---
通过阅读本文档，用户可以在地平线X3开发板上轻松抓取mipi摄像头的视频流数据，并通过ROS平台发布满足ROS标准的图片数据，供其他ROS Node订阅获取。目前支持F37、IMX415 mipi标准设备。
Mipi_cam Node package是地平线机器人开发平台的一部分，基于地平线VIO和ROS2 Node进行二次开发，为应用开发提供简单易用的摄像头数据采集功能的功能，避免重复开发获取视频的工作。支持 share mem 方式发布。

# Build
---
## Dependency

依赖库：

- opencv:3.4.5

ros package：
- image_transport_plugins
- camera_info_manager
- cv_bridge
- sensor_msgs
- hbm_img_msgs

其中 image_transport_plugins/camera_info_manager/cv_bridge 为ROS开源的package，需要手动安装，具体安装方法：
```
# 方法1:使用apt 安装：
sudo apt-get install ros-foxy-camera-info-manager 
sudo apt-get install ros-foxy-image-transport-plugins
sudo apt-get install ros-foxy-cv-bridge -y
# 方法2：使用 rosdep检查并自动安装pkg依赖，由于使用的是国外源会导致概率性的安装失败，建议选用第一种：
#安装 rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
#在ros 的工程路径下执行安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖：
rosdep install -i --from-path . --rosdistro foxy -y
```

## 开发环境
- 编程语言：C/C++
- 开发平台：X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链：Linux GCC 9.3.0/Linaro GCC 9.3.0
## package说明
---
源码包含mipi_cam package。mipi_cam 编译完成后，头文件、动态库以及依赖安装在install/mipi_cam 路径。

## 编译
支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。
### 编译选项

1、IMAGE_TRANSPORT_PKG

- image_transport_plugin pkg依赖的使能开关，默认关闭（OFF），编译时使用-DIMAGE_TRANSPORT_PKG=ON命令打开。
- 如果打开，编译和运行会依赖image_transport相关 pkg，支持发布compressed topic。
- 如果关闭，编译和运行不依赖image_transport相关 pkg，只支持发布原始图片。

2、SHARED_MEM

- shared mem（共享内存传输）使能开关，默认关闭（OFF），编译时使用-DSHARED_MEM=ON命令打开。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- 对于shared mem通信方式，当前不支持compressed topic。

### X3 Ubuntu系统上编译
1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已依赖pkg ，详见 Dependency 部分

2、编译：
  - 只发布支持share mem的 hbmem_img主题的图片：`colcon build --packages-select mipi_cam --cmake-args -DIMAGE_TRANSPORT_PKG=OFF -DSHARED_MEM=ON`
  - 支持发布压缩格式图片：`colcon build --packages-select mipi_cam`或`colcon build --packages-select mipi_cam --cmake-args -DIMAGE_TRANSPORT_PKG=ON`。


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
  
  colcon build --packages-select mipi_cam \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
     -DSYS_ROOT=/mnt/test/cc_ws/sysroot_docker \
     -DIMAGE_TRANSPORT_PKG=OFF \
     -DSHARED_MEM=ON
- 编译选项中关闭了 image_transport pkg依赖，打开了shared mem通信方式，只支持发布 hbmem_img 主题的图片。
     
  ```

- 其中SYS_ROOT为交叉编译系统依赖路径，此路径具体地址详见第1步“编译环境确认”的交叉编译说明。


# Usage
用户直接调用ros2 命令启动即可：

```
# 使能sensor mclk
echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart
echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host1/param/snrclk_freq
echo 1 > /sys/class/vps/mipi_host0/param/snrclk_en
echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq
# 添加sensor库路径, 或者将sensor库拷贝到系统库路径cp install/mipi_cam/sensorlib/lib* /usr/lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:install/mipi_cam/sensorlib

export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
# 默认F37 sensor
ros2 run mipi_cam mipi_cam
```

node会发布/image_raw和/image_raw/compressed两个topic，分别对应rgb8和mjpeg格式图片，使用 share mem 发布主题：hbmem_img

利用 rqt_image_view 可以查看发布的图片主题，也可以用图片消费节点。例如：这个repo下的example去直接获取图片进行推理等应用。

可以设置使用的sensor，发布图片的编码方式和分辨率。

使用video_device参数设置使用的sensor。目前支持 F37（默认），IMX415（通过--ros-args -p video_device:=IMX415设置），F37 默认分辨率是1920x1080；IMX415 是3840x2160。

使用image_width和image_height参数设置发布图片的分辨率:

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用out_format参数设置发布图片的编码方式，默认是rgb8和mjpeg编码方式，支持nv12格式（/image_raw topic），例如使用F37 sensor发布960x540分辨率的nv12格式图片：

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用 io_method 参数设置发布图像采用的方式，目前 hbmem 发布的主题是固定的：hbmem_img

`ros2 run mipi_cam mipi_cam --ros-args -p io_method:=hbmem`

---

