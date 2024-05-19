#!/bin/bash

docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network scripts_my_bridge_network --device=/dev/usb_rear_wheel --device=/dev/usb_robot_arm  --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
