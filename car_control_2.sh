docker run -it --rm -v "$(pwd)/src:/workspaces/src" --device=/dev/usb_rear_wheel --network host --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
