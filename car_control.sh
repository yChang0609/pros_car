#!/bin/bash

# 初始化device參數
device_options=""

# 檢查設備動態加入
if [ -e /dev/usb_front_wheel ]; then
    device_options+=" --device=/dev/usb_front_wheel"
fi

if [ -e /dev/usb_rear_wheel ]; then
    device_options+=" --device=/dev/usb_rear_wheel"
fi

if [ -e /dev/usb_robot_arm ]; then
    device_options+=" --device=/dev/usb_robot_arm"
fi

if [ -e /dev/video0 ]; then
    device_options+=" --device=/dev/video0"
fi

if [ -e /dev/video1 ]; then
    device_options+=" --device=/dev/video1"
fi

if [ -e /dev/imu_usb ]; then
    device_options+=" --device=/dev/imu_usb"
fi

# 建構完整指令
docker run -it --rm -v "$(pwd)/src:/workspaces/src" --device /dev/bus/usb:/dev/bus/usb --network scripts_my_bridge_network $device_options --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
