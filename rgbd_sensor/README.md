# Getting Started with rgbd_sensor Node
---
# Intro
---
通过阅读本文档，用户可以在地平线X3开发板上轻松抓取 mipi 深度摄像头的视频流数据，并通过ROS平台发布满足ROS标准的深度图数据/灰度图数据/RGB图像数据/相机内参，还有经过运算后的点云数据，供其他ROS Node订阅获取，可以在 rviz 上观看实时效果。支持 share mem 方式发布。

# Build
---
## Dependency

依赖库：
ros package：
- sensor_msgs
- hbm_img_msgs

hbm_img_msgs pkg是在hobot_msgs中自定义的图片消息格式，用于shared mem场景下的图片传输。

## 开发环境
- 编程语言：C/C++
- 开发平台：X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链：Linux GCC 9.3.0/Linaro GCC 9.3.0
## package说明
---
源码包含 rgbd_sensor package。rgbd_sensor 编译完成后，头文件、动态库以及依赖安装在install/rgbd_sensor 路径。

## 编译
支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。

### X3 Ubuntu系统上编译
1、编译环境确认

- 板端已安装X3 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
- 已依赖pkg ，详见 Dependency 部分

2、编译：
  `colcon build --packages-select rgbd_sensor`。


### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台 robot_dev_config repo中的README.md。
- 已编译 hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令： 

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select rgbd_sensor \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  
  ```


# Usage

## 目前参数列表：

| 参数名      | 含义                 | 取值                          | 默认值                |
| ----------- | -------------------- | ----------------------------- | --------------------- |
| sensor_type   | 设备类型            | 字符串，目前只支持舜宇 CP3AM    |      CP3AM               |
| io_method     | 输出数据传输的方式   | ros/shared_mem               |      ros               |
| color_width   |  模组输出图像宽      |         1920                 |         1920           |
| color_height  |  模组输出图像高      |         1080                 |         1080           |
| color_fps     |  模组输出图像帧率    |         10                   |         10            |
| enable_color  |  是否发布图像        |         True/False          |         True          |
| color_width   |  模组输出图像宽      |         1920                 |         1920           |
| color_height  |  模组输出图像高      |         1080                 |         1080           |
| color_fps     |  模组输出图像帧率    |         10                   |         10            |
| enable_color  |  是否发布图像        |         True/False          |         True          |
| depth_width   |  模组输出深度图像宽      |         224                 |         224           |
| depth_height  |  模组输出深度图像高      |         129                 |         129           |
| depth_fps     |  模组输出深度图像帧率    |         10                   |         10            |
| enable_depth  |  是否发布深度图像        |         True/False          |         True          |
| enable_pointcloud  |  是否发布点云        |         True/False          |         True          |
| enable_aligned_pointcloud  |  是否发布标定点云        |         True/False          |         True          |
| infra_width   |  模组输出灰度图像宽      |         224                 |         224           |
| infra_height  |  模组输出灰度图像高      |         108                 |         108           |
| infra_fps     |  模组输出灰度图像帧率    |         10                   |         10            |
| enable_infra  |  是否发布灰度图像        |         True/False          |         True          |
| camera_calibration_file_path  | 相机标定文件的存放路径  | 根据实际的相机标定文件存放路径配置  | /opt/tros/lib/rgbd_sensor/config/CP3AM_calibration.yaml |

目前舜宇模组，只能输出 1080P 的标定，所以图像参数目前没有什么实际效果，都是默认值。
发布主题包括：
```
#ros
#深度图
/rgbd_CP3AM/depth/image_rect_raw
#点云
/rgbd_CP3AM/depth/color/points
#标定点云
/rgbd_CP3AM/aligned_depth_to_color/color/points
#灰度图
/rgbd_CP3AM/infra/image_rect_raw
#颜色图
/rgbd_CP3AM/color/image_rect_raw
#相机内参
/rgbd_CP3AM/color/camera_info
#shared mem:
#颜色图
hbmem_img
#深度图
hbmem_depth
#灰度图
hbmem_infra
```
## 注意：
 1：在当前目录 cp -r install/${PKG_NAME}/lib/${PKG_NAME}/parameter/ .，其中 ${PKG_NAME} 为具体的package名。

 2：需要把校准库 install/lib/libgc2053_linear.so 拷贝到：/lib/sensorlib/

 3：读取相机内参文件失败时，会出现无法发布相机内参的警告，但不影响rgbd_sensor的其他功能

 4：相机标定文件的读取路径默认为：/opt/tros/lib/rgbd_sensor/config/CP3AM_calibration.yaml，请确认文件路径正确，相机内参发布话题名为：/rgbd_CP3AM/color/camera_info
## X3 Ubuntu系统
用户直接调用ros2 命令启动即可：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh

ros2 run rgbd_sensor rgbd_sensor
```
传参数方式：
`ros2 run rgbd_sensor rgbd_sensor --ros-args --log-level info --ros-args -p io_method:=ros`


运行方式2，使用launch文件启动：
`ros2 launch install/share/rgbd_sensor/launch/rgbd_sensor.launch.py`

## X3 linaro系统

把在docker 交叉编译的install 目录拷贝到linaro 系统下，例如:/userdata
需要首先指定依赖库的路径，例如：
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`


修改 ROS_LOG_DIR 的路径，否则会创建在 /home 目录下，需要执行 mount -o remount,rw /，才可以在 /home 下创建日志
`export ROS_LOG_DIR=/userdata/`

运行 rgbd_sensor
```
// 默认参数方式
/userdata/install/lib/rgbd_sensor/rgbd_sensor
// 传参方式
#/userdata/install/lib/rgbd_sensor/rgbd_sensor --ros-args -p io_method:=ros

```
# 结果分析
## X3结果展示
相机运行正常并成功发布相机内参，则输出以下信息
```
[INFO] [1662470301.451981102] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.461459373] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.528991765] [rgbd_node]: publish camera info.

[INFO] [1662470301.533941164] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.543212650] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.608683161] [rgbd_node]: publish camera info.

[INFO] [1662470301.613038915] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.621678967] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.713615364] [rgbd_node]: publish camera info.

[INFO] [1662470301.717811416] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.726191436] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.818197996] [rgbd_node]: publish camera info.

[INFO] [1662470301.822495336] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.831028892] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470301.909490084] [rgbd_node]: publish camera info.

[INFO] [1662470301.913704051] [rgbd_node]: [pub_ori_pcl]->pub pcl w:h=24192:1,nIdx-24192:sz=24192.
[INFO] [1662470301.922111820] [rgbd_node]: [timer_ros_pub]->pub dep w:h=224:129,sz=982464, infra w:h=224:108, sz=24192.
[INFO] [1662470302.014477201] [rgbd_node]: publish camera info.
```

## rviz2 观看效果
安装：apt install ros-foxy-rviz-common ros-foxy-rviz-default-plugins ros-foxy-rviz2