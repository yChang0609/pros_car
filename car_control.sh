#!/bin/bash

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# GPU Flags initialization
GPU_FLAGS=""
OS_TYPE=$(uname -s)

if [ "$OS_TYPE" = "Linux" ]; then
    if [ -f "/etc/nv_tegra_release" ]; then
        GPU_FLAGS="--runtime nvidia"
    else
        GPU_FLAGS="--gpus all"
    fi
elif [ "$OS_TYPE" = "Darwin" ]; then
    GPU_FLAGS=""
else
    GPU_FLAGS="--gpus all"
fi

# Check if GPU support is available
if [ -n "$GPU_FLAGS" ]; then
    if ! docker info --format '{{json .}}' | grep -q '"Runtimes".*nvidia'; then
        echo "Warning: GPU support is not available. Removing GPU flags."
        GPU_FLAGS=""
    fi
fi

echo "Detected OS: $OS_TYPE"
echo "GPU Flags: $GPU_FLAGS"

# Initialize device options
device_options=""

# Check for dynamically added devices
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

if [ -e /dev/bus/usb ]; then
    device_options+=" --device=/dev/bus/usb"
fi

# run docker container
docker run -it --rm \
    -v "$(pwd)/src:/workspaces/src" \
    --network compose_my_bridge_network \
    --env-file ./.env \
    $PORT_MAPPING \
    $GPU_FLAGS \
    $device_options \
    registry.screamtrumpet.csie.ncku.edu.tw/alianlbj23/pros_car_docker_image:latest \
    /bin/bash
