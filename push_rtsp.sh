#!/bin/bash
killall ./bin/mediamtx
gnome-terminal -- bash -c "./bin/mediamtx; exec bash"
ffmpeg -framerate 60 -video_size 424x240 -i /dev/video4 -vcodec libx264 -preset ultrafast -tune zerolatency -rtsp_transport udp -f rtsp rtsp://127.0.0.1:2212/video
