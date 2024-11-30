#!/bin/bash

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# # gpu check
# if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
#     echo "NVIDIA GPU detected"
#     GPU_FLAGS="--runtime nvidia --gpus all"
# else
#     echo "No NVIDIA GPU detected or NVIDIA drivers not installed"
#     GPU_FLAGS=""
# fi

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

# run docker container
docker run -it --rm \
    -v "$(pwd)/src:/workspaces/src" \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network compose_my_bridge_network \
    --env-file ./.env \
    $PORT_MAPPING \
    --runtime nvidia \
    $device_options \
    ghcr.io/otischung/pros_ai_image_pros_car:latest \
    /bin/bash