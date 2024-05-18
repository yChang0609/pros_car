docker run -it --rm -p 9090:9090 -v "$(pwd)/src:/workspaces/src" --network scripts_my_bridge_network --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
