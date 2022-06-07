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
