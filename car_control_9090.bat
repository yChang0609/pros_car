@echo off
setlocal

rem Initialize device options
set "device_options="

rem Check for dynamically added devices
if exist /dev/usb_front_wheel (
    set "device_options=%device_options% --device=/dev/usb_front_wheel"
)

if exist /dev/usb_rear_wheel (
    set "device_options=%device_options% --device=/dev/usb_rear_wheel"
)

if exist /dev/usb_robot_arm (
    set "device_options=%device_options% --device=/dev/usb_robot_arm"
)

if exist /dev/video0 (
    set "device_options=%device_options% --device=/dev/video0"
)

if exist /dev/video1 (
    set "device_options=%device_options% --device=/dev/video1"
)

if exist /dev/imu_usb (
    set "device_options=%device_options% --device=/dev/imu_usb"
)

if exist /dev/bus/usb (
    set "device_options=%device_options% --device=/dev/bus/usb"
)

rem Construct and run the Docker command
docker run -it --rm ^
    -v "%cd%/src:/workspaces/src" ^
    --network scripts_my_bridge_network ^
    -p 9090:9090 ^
    %device_options% ^
    --env-file ./.env ^
    ghcr.io/otischung/pros_ai_image:latest ^
    /bin/bash
