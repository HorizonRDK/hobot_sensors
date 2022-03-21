# Getting Started with Mipi_Cam Node
---
# Intro
---
通过阅读本文档，用户可以在地平线X3开发板上轻松抓取mipi摄像头的视频流数据，并通过ROS平台发布满足ROS标准的图片数据，供其他ROS Node订阅获取。目前支持F37、IMX415 mipi标准设备。
Mipi_cam Node package是地平线机器人开发平台的一部分，基于地平线VIO和ROS2 Node进行二次开发，为应用开发提供简单易用的摄像头数据采集功能的功能，避免重复开发获取视频的工作。
支持share mem
# Build
---

## 开发环境
- 编程语言：C/C++
- 开发平台：X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链：Linux GCC 9.3.0/Linaro GCC 9.3.0
## package说明
---
源码包含mipi_cam package。mipi_cam 编译完成后，头文件、动态库以及依赖安装在install/mipi_cam 路径。
-DTROS=OFF表示不使用share mem，-DTROS=ON表示使用share mem

## 编译
编译环境确认：
- 当前编译终端设置ROS环境变量：source /opt/ros/foxy/setup.bash。
- 已安装ROS2编译工具colcon，安装的ROS不包含编译工具colcon，需要手动安装colcon。命令：`apt update ;apt install python3-colcon-common-extensions`。
- 支持两种编译方式：
  - 只发布nv12格式图片，不依赖第三方pkg
  - 支持发布压缩格式图片，依赖第三方pkg

- 编译：
  - 只发布nv12格式图片：`colcon build --packages-select mipi_cam --cmake-args -DIMAGE_TRANSPORT_PKG=OFF`
  - 支持发布压缩格式图片：`colcon build --packages-select mipi_cam`或`colcon build --packages-select mipi_cam --cmake-args -DIMAGE_TRANSPORT_PKG=ON`。
  - 发布压缩格式图片依赖的开源package，需要手动安装：


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

node会发布/image_raw和/image_raw/compressed两个topic，分别对应rgb8和mjpeg格式图片。

利用 rqt_image_view 可以查看发布的图片主题，也可以用图片消费节点。例如：这个repo下的example去直接获取图片进行推理等应用。

可以设置使用的sensor，发布图片的编码方式和分辨率。

使用video_device参数设置使用的sensor。目前支持 F37（默认），IMX415（通过--ros-args -p video_device:=IMX415设置），F37 默认分辨率是1920x1080；IMX415 是3840x2160。

使用image_width和image_height参数设置发布图片的分辨率:

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用out_format参数设置发布图片的编码方式，默认是rgb8和mjpeg编码方式，支持nv12格式（/image_raw topic），例如使用F37 sensor发布960x540分辨率的nv12格式图片：

`ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=540 -p video_device:=F37`

使用 io_method 参数设置发布图像采用的方式，目前 hbmem 发布的主题是固定的：hbmem_img

`ros2 run mipi_cam mipi_cam --ros-args -p io_method:=hbmem`

sub端也需要指明topic 为 hbmem_img，才可以接收 hbmem 传过来的数据：

`ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=hbmem_img`
---

