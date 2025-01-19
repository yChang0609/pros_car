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
    # Check for NVIDIA runtime
    if [ -f "/etc/nv_tegra_release" ]; then
        GPU_FLAGS="--runtime nvidia"
    elif docker info --format '{{json .}}' | grep -q '"Runtimes".*nvidia'; then
        GPU_FLAGS="--gpus all"
    fi
fi

# Verify GPU support with nvidia-container-cli
USE_GPU=false
if [ -n "$GPU_FLAGS" ]; then
    if nvidia-container-cli info > /dev/null 2>&1; then
        USE_GPU=true
    else
        echo "Warning: GPU support detected but not usable. Skipping GPU configuration."
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

# Function to test if Docker can run with GPU
test_gpu_flags() {
    docker run --rm $GPU_FLAGS registry.screamtrumpet.csie.ncku.edu.tw/alianlbj23/pros_car_docker_image:latest /bin/bash -c "echo GPU test" > /dev/null 2>&1
}

# Check if GPU flags cause an error
if [ "$USE_GPU" = true ]; then
    echo "Testing Docker run with GPU flags..."
    if ! test_gpu_flags; then
        echo "Error: Running Docker with GPU flags failed. Disabling GPU flags."
        GPU_FLAGS=""
        USE_GPU=false
    fi
fi

# Run docker container based on GPU support
if [ "$USE_GPU" = true ]; then
    echo "Running Docker container with GPU support..."
    docker run -it --rm \
        -v "$(pwd)/src:/workspaces/src" \
        --network compose_my_bridge_network \
        --env-file ./.env \
        $PORT_MAPPING \
        $GPU_FLAGS \
        $device_options \
        registry.screamtrumpet.csie.ncku.edu.tw/alianlbj23/pros_car_docker_image:latest \
        /bin/bash
else
    echo "Running Docker container without GPU support..."
    docker run -it --rm \
        -v "$(pwd)/src:/workspaces/src" \
        --network compose_my_bridge_network \
        --env-file ./.env \
        $PORT_MAPPING \
        $device_options \
        registry.screamtrumpet.csie.ncku.edu.tw/alianlbj23/pros_car_docker_image:latest \
        /bin/bash
fi
