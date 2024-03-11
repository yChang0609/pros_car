xhost +
docker run -it --rm -v "$(pwd)/src:/workspaces/src" --network pros_app_my_bridge_network  --env-file ./.env -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ghcr.io/otischung/pros_ai_image:latest /bin/bash
