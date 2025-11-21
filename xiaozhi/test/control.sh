#!/usr/bin/env bash

PORT="/dev/ttyACM0"
BAUD=115200

declare -A CMD=(
    ["go"]="\x41"
    ["right"]="\x43"
    ["back"]="\x45"
    ["left"]="\x47"
    ["stop"]="\x5A"
    ["speedup"]="\x58"
    ["speeddown"]="\x59"
)

if [ $# -ne 1 ]; then
    echo "用法: $0 {go|back|left|right|speedup|speeddown|stop}"
    exit 1
fi

state=$1

if [[ -z "${CMD[$state]}" ]]; then
    echo "错误指令: $state"
    exit 1
fi

stty -F "$PORT" $BAUD raw -echo

printf "${CMD[$state]}" > "$PORT"

echo "已发送: $state"
