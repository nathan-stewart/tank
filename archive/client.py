#!/usr/bin/env pythnon3
import zmq
import sys
import time
from smbus2 import SMBus  # requires smbus2, a Python 3.x library
from motor_driver import MotorDriver
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

time = 0
integral = 0
time_prev = -1e-6
e_prev = 0
motor = MotorDriver()
def PID(Kp, Ki, Kd, setpoint, measurement):
    global time, integral, time_prev, e_prev

    # Value of offset - when the error is equal zero
    offset = 320

    # PID calculations
    e = setpoint - measurement

    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)

    # calculate manipulated variable - MV
    MV = offset + P + integral + D

    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

# The chassis uses the motor model JGB3865-520R45-12. In this designation,
# 'J' stands for a DC motor, 'GB' signifies an eccentric output shaft, '38' denotes
# the gearbox diameter, '520' represents the motor model, 'R45' indicates a
# reduction ratio of 1:45, and '12' denotes the rated voltage of 12V. The interface
# specifications are illustrated in the diagram below:
# https://www.hiwonder.com.cn/store/learn/142.html


# Initialize I2C bus
bus = SMBus(1)  # usually 1 for Raspberry Pi/Jetson Nano

# Example control loop
#while True:
    # Measure the process variable
#    current_value = measure_process()

    # Calculate the control output
#    control = pid(current_value)

    # Apply the control output to your system
 #   apply_control(control)


# Server (e.g., on the robot)
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

motor_left = 0
motor_right = 0
lurd = 0
while True:
    message = socket.recv_string()
    socket.send_string("Ack")
    event,key = message.split(":")
    mask = 0b0000
    if key == "Up":
        mask = 0b0100
    if key == "Down":
        mask = 0b0001
    if key == "Left":
        mask = 0b1000
    if key == "Right":
        mask = 0b0010

    if event == "Pressed":
        lurd |= mask
    if event == "Released":
        lurd &= ~mask
    #print(f"lurd: {lurd:04b}")
    if lurd == 0b0100: # up
        motor_left = 100
        motor_right = 100
    elif lurd == 0b0001: # down
        motor_left = -100
        motor_right = -100
    elif lurd == 0b1000: # left
        motor_left = -20
        motor_right = 20
    elif lurd == 0b0010: # right
        motor_left = 20
        motor_right = -20
    elif lurd == 0b0110: # up right
        motor_left = 100
        motor_right = 50
    elif lurd == 0b0101: # up left
        motor_left = 50
        motor_right = 100
    elif lurd == 0b1100: # down right
        motor_left = 0
        motor_right = 100
    elif lurd == 0b1010: # down left
        motor_left = 100
        motor_right = 0
    else:
        motor_left = 0
        motor_right = 0
    print(f"Motor left: {motor_left}, Motor right: {motor_right}")
    motor.set_tracks([motor_left, motor_right])


