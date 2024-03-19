@echo off
docker run -it --rm  --name pros_car -v "%cd%/src:/workspaces/src" -v "%cd%/FUNAI_digitaltwin_avoidance:/workspaces/FUNAI_digitaltwin_avoidance" --network scripts_my_bridge_network --env-file ./.env ghcr.io/otischung/pros_ai_image:latest /bin/bash
