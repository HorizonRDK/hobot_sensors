Getting Started with image_subscribe_example
=======


# Intro

image_subscribe_example package用于显示接收 ROS2 Node 发布的image msg。

# Build
colcon build --packages-select image_subscribe_example

## Dependency

ros package：
- cv_bridge
- sensor_msgs

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

- CMakeLists.txt中指定Mipi_cam package的安装路径，默认为`../install/image_subscribe`。
- 编译：`colcon build --packages-select image_subscribe_example`

# Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run image_subscribe_example subscribe_example
#可以输入参数进行订阅并保存图片：默认是不保存图片，除非设置了save_dir参数
ros2 run image_subscribe_example subscribe_example --ros-args -p sub_img_topic:=/image_raw/compressed -p save_dir:=/userdata
```
