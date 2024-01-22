docker run -it --rm -v "$(pwd)/src:/src" --device=/dev/usb_rear_wheel --network host --env-file ./.env ghcr.io/otischung/pros_car:latest /bin/bash
