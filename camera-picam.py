#!/usr/bin/env ptyhon3
from picamera2 import Picamera2
import subprocess
import numpy as np

# Create Picamera2 instance
picam2 = Picamera2()

# Configure the camera to capture 2592x1944 resolution
camera_config = picam2.create_video_configuration(
    main={"size": (2592, 1944)},
    controls={"FrameRate": 15}  # Adjust frame rate as needed
)

picam2.configure(camera_config)
picam2.start()

# GStreamer pipeline setup
gst_cmd = [
    "gst-launch-1.0", 
    "appsrc", "do-timestamp=true", "format=3", "is-live=true", 
    "caps=video/x-raw,format=RGB,width=2592,height=1944,framerate=30/1",
    "!", "videoconvert",
    "!", "videoscale",
    "!", "video/x-raw,width=1280,height=720",  # Resize to 1280x720
    "!", "x264enc", "tune=zerolatency", "speed-preset=superfast", "bitrate=500",
    "!", "rtph264pay", "config-interval=1",
    "!", "udpsink", "host=0.0.0.0", "port=5000"  # Use localhost for testing
]

# Start GStreamer process
process = subprocess.Popen(gst_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)

try:
    while True:
        # Capture a frame
        frame = picam2.capture_array("main")
        
        if frame is not None:
            # Convert frame to bytes and send to GStreamer pipeline
            # Convert the image data to the expected BGRx format
            frame_bgr = frame[..., ::-1]  # Convert RGB to BGR
            process.stdin.write(frame_bgr.tobytes())
            process.stdin.flush()
        else:
            print("Frame is None. Skipping...")
finally:
    picam2.stop()
    process.terminate()
