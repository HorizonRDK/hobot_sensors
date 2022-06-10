# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
              package='hobot_usb_cam',
              executable='hobot_usb_cam',
              name='hobot_usb_cam',
              parameters=[
                          {"frame_id": "default_usb_cam"},
                          {"framerate": 30},
                          {"image_height": 480},
                          {"image_width": 640},
                          {"io_method": "mmap"},
                          {"pixel_format": "mjpeg"},
                          {"video_device": "/dev/video8"},
                          {"zero_copy": False}
                         ]
            )
    ])
