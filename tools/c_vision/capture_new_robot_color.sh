#!/usr/bin/env bash
set -e

restart_cheese() {
    DISPLAY=:0 nohup cheese >/tmp/cheese_restart.log 2>&1 </dev/null &
}
trap restart_cheese EXIT

pkill -TERM -x cheese || true
sleep 2
timeout 12 ffmpeg -hide_banner -loglevel error -nostdin -f v4l2 \
    -video_size 1280x720 -pixel_format yuyv422 -i /dev/video4 \
    -frames:v 1 -y /tmp/cake_color_raw_20261122.png
