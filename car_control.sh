#!/bin/bash

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# 檢查操作系統並初始化 GPU 標誌
GPU_FLAGS=""
OS_TYPE=$(uname -s)

if [ "$OS_TYPE" = "Linux" ]; then
    # 如果是 Jetson 平台，使用 --runtime nvidia
    if [ -f "/etc/nv_tegra_release" ]; then
        GPU_FLAGS="--runtime nvidia"
    else
        GPU_FLAGS="--gpus all"
    fi
elif [ "$OS_TYPE" = "Darwin" ]; then
    # macOS 平台不需要 GPU 標誌
    GPU_FLAGS=""
else
    # 默認假設是 Windows WSL 或其他 Linux 平台
    GPU_FLAGS="--gpus all"
fi

echo "Detected OS: $OS_TYPE"
echo "GPU Flags: $GPU_FLAGS"

# 初始化 device 參數
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
    --network compose_my_bridge_network \
    --env-file ./.env \
    $PORT_MAPPING \
    $GPU_FLAGS \
    $device_options \
    registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_jetson_driver_image_pros_car:latest \
    /bin/bash
