#!/usr/bin//python3
import zmq
import threading
import time
import subprocess
import atexit
from motor_driver import MotorDriver
#import qwiic_icm20948
import smbus2 as smbus
import numpy as np
#from PID_Py.PID import PID
#from filterpy.kalman import KalmanFilter

threads = []
running  = True

# start/stop camera with app
camera = None
def stop_camera():
    global camera 
    if camera:
        camera.terminate()
        camera.wait()
        camera = None

def start_camera(host):
    global camera
    print(f'Connect message from {host} - starting camera')
    camera = subprocess.Popen(f'libcamera-vid -t 0 --width 1920 --height 1080 --framerate 10 --codec h264 --inline -o udp://{host}:5000', shell=True)
atexit.register(stop_camera)

i2c_bus = smbus.SMBus(1)
i2c_lock = threading.Lock()
imu_lock = threading.Lock()
kf_lock  = threading.Lock()

motor = MotorDriver(i2c_lock, i2c_bus)
motor.set_tracks([0, 0])

running = True


def set_motor_drive(msg):
    global motor
    motor_left, motor_right = [int(x) for x in data.split(",")]
    motor.set_tracks([motor_left, motor_right])

def dump_sensors():
    global motor
    global running
    while running:
        motor.print()
        time.sleep(1.0)

threads.append(threading.Thread(target=dump_sensors))
for thread in threads:
    thread.start()

try:
    # Server (e.g., on the robot)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 seconds timeout
    socket.bind("tcp://*:5555")

    while running:
        message = None
        try:
            message = socket.recv_string()
        except zmq.Again:
            continue
        
        if message:
            socket.send_string("Ack")
            event,data = message.split(":")
            if event == "drive":
                set_motor_drive(data)
            elif event == "connect":
                start_camera(host=data)
            elif event == "disconnect":
                stop_camera()
                running = False
except:
    print("Uncaught exception - bailing out")

print('exiting')
running = False
socket.close()
context.term()
motor.stop()

while threads:
    threads.pop().join()
