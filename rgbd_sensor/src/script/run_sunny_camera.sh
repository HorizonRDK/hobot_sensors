#!/bin/sh

service adbd stop
#/etc/init.d/usb-gadget.sh start uvc isoc
/etc/init.d/usb-gadget.sh start uvc

cd /app/sunnytest
export LD_LIBRARY_PATH='/lib/sensorlib/:./libs'

chmod +x sunny_camera
./sunny_camera &

