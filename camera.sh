#!/usr/bin/env bash
libcamera-vid -t 0 --width 1920 --height 1080 --framerate 30 --codec h264 --inline -o udp://192.168.1.131:5000

