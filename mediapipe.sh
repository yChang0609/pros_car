xhost +
docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network scripts_my_bridge_network  --gpus=all --env-file ./.env -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ghcr.io/otischung/pros_ai_image:latest /bin/bash
