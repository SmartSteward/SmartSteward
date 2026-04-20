#!/usr/bin/env bash
set -e

export DISPLAY=:0
export XAUTHORITY=/home/aaa123/.Xauthority

# Start XiaoZhi in Web mode.
cd /home/aaa123/SmartSteward/xiaozhi && /home/aaa123/SmartSteward/xiaozhi/.venv/bin/python main.py --mode web &

sleep 15

firefox --kiosk http://127.0.0.1:42249 &
xdotool mousemove 0 0
