# Hobot_USB_CAM

# 功能介绍

从USB摄像头获取图像数据并通过image/hbmem_image topic发布

# 编译

## 依赖库

ros package：

- rclcpp
- sensor_msgs
- hbm_img_msgs
- v4l-utils

hbm_img_msgs为自定义消息格式，用于发布shared memory类型图像数据，定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select hobot_usb_cam
`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select hobot_usb_cam \
  --merge-install \
  --cmake-force-configure \
  --cmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
      -DTHIRDPARTY=ON \
      -DBUILD_TESTING:BOOL=OFF \
      -DCMAKE_BUILD_RPATH="`pwd`/build/poco_vendor/poco_external_project_install/lib/;`pwd`/build/libyaml_vendor/libyaml_install/lib/"
```

## 注意事项

# 使用介绍

## 依赖

websocket接收图像消息和智能结果消息，根据时间戳进行匹配，然后输出给web端渲染显示，也可单独显示图像。

图像消息支持`sensor_msgs::msg::Image`以及`shared_mem`的`hbm_img_msgs::msg::HbmMsg1080P`类型消息，必须为hobot codec输出的jpeg格式数据。

智能结果消息支持`ai_msgs::msg::PerceptionTargets`类型消息，其中`header.stamp`必须和该结果对应的image消息相同，websocket会使用该字段进行消息匹配，还有智能结果对应的宽高必须要和接收到的图像分辨率一致。

具体依赖的package有：

- mipi_cam：启动mipi cam，发布nv12类型图像消息
- hobot_codec：将mipi_cam发布的nv12图像编码为websocket需要的jpeg格式图像
- mono2d_body_detection：接收nv12格式数据，进行算法推理，发布人体、人头、人脸、人手框感知消息

## 参数

| 参数名      | 解释             | 类型   | 支持的配置                 | 是否必须 | 默认值             |
| ------------| -----------------| -------| --------------------------| -------- | -------------------|
| frame_id    | 消息标志符       | string | 根据需要设置frame_id名字   | 否       | "default_usb_cam"  |
| framerate   | 帧率             | int    | 根据sensor支持选择         | 否       | 30                 |
| image_height| 图像高方向分辨率 | int    | 根据sensor支持选择         | 否       | 640                |
| image_width | 图像宽方向分辨率 | int    | 根据sensor支持选择         | 否        | 480               |
| io_method   | io类型           | string | mmap/read/userptr          | 否       | “mmap”            |
| pixel_format| 像素格式         | string | 当前只支持mjpeg            | 否        | “mjpeg”           |
| video_device| 设备驱动名称     | string | 设备名称一般为/dev/videox  | 是        | “/dev/video0”     |
| zero-copy   | 使能“zero-copy”  | bool   | true/false                 | 否       | “false”           |


## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

source setup.bash

~~~shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
~~~

使用launch运行hobot_usb_cam
~~~shell
ros2 launch  hobot_usb_cam hobot_usb_cam.launch.py
~~~
使用命令运行hobot_usb_cam
~~~shell
ros2 run hobot_usb_cam hobot_usb_cam --ros-args --log-level info --ros-args -p video_device:="/dev/video8"
~~~

注意：video_device参数需要根据实际情况配置
### **Linux**

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

./install/lib/hobot_usb_cam/hobot_usb_cam --ros-args --log-level info --ros-args -p video_device:="/dev/video8"

## 注意事项

目前使用“zero-copy”仅支持1920*1080、960*540、640*480三种分辨率图像，如果需要使用其他分辨率，需要自行创建对应的ros message。

当配置的分辨率硬件不支持时，会自动选择接近的分辨率进行图像获取

# 结果分析

## X3结果展示

```
root@ubuntu:~# ros2 run hobot_usb_cam hobot_usb_cam --ros-args --log-level info --ros-args -p video_device:="/dev/video8"

[INFO] [1653875464.486111221] [hobot_usb_cam]: Set resolution to 640x480

[INFO] [1653875464.488402126] [hobot_usb_cam]: Set framerate to be 30

[INFO] [1653875465.134289369] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.138189789] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.172409484] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.176569685] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.212285274] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.255996155] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.296020731] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

[INFO] [1653875465.336038058] [hobot_usb_cam]: publish image 640x480 encoding:2 size:614400

```

# 常见问题
