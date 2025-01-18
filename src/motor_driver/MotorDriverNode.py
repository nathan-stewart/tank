#!/usr/bin/env python3
import struct
from collections import deque
import smbus2 as smbus
from time import sleep
import threading

# Hiwonder motor driver module
MC_I2C_ADDR = 0x34  # I2C address of the motor controller
ADC_BAR_ADDR=0x00                   # ADC battery sampling
MOTOR_TYPE_ADDR=0x14                # Encoder motor type settings
MOTOR_ENCODER_POLARITY_ADDR=0x15    # Encoder direction polarity settings
MOTOR_FIXED_PWM_ADDR=0x1f           # Fixed PWM control
                                    # open-loop control
                                    # Value Range: -100 to 100
MOTOR_FIXED_SPEED_ADDR=0x33         # Closed Loop speed control.
MOTOR_ENCODER_TOTAT_ADDR=0x3c       # The total pulse values for the four encoded motors.

# motor type - Suspension Crawler motors are JGB37-520-12V-110RPM (type 3)
MOTOR_TYPE_WITHOUT_ENCODER = 0      # No encoder present
MOTOR_TYPE_TT = 1                   # TT encoder motor
MOTOR_TYPE_N20 = 2                  # N20 encoder motor
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 # the magnetic ring generates 44 pulses per revolution,
                                    # combined with a gear reduction ratio of 90 Default

# Sparkfun Qwiic Pinout
# https://learn.sparkfun.com/tutorials/qwiic-cable-hat-hookup-guide/all
# Black - GND
# Red - 3.3V
# Yellow - SDA
# Blue - SCL

I2C_BUS = 1
class MotorDriver:
    def __init__(self, i2c_lock, bus):
        self.bus = bus
        self.i2c_lock = i2c_lock
        with i2c_lock:
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_TYPE_ADDR,              MOTOR_TYPE_JGB37_520_12V_110RPM)
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR,  0)
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR,       0)

    def stop(self):
        with self.i2c_lock:
            self.bus.write_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, [0, 0, 0, 0])

    def set_tracks(self, tracks):
        with self.i2c_lock:
            state = self.bus.read_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, 4)
            self.bus.write_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, [tracks[0], tracks[1], state[2], state[3]])

    def set_turret(self, turret):
        with self.i2c_lock:
            state = self.bus.read_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, 4)
            self.bus.write_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, state[0],state[1], turret[0], turret[1])


    def read_battery(self):
        with self.i2c_lock:
            lsb,msb = self.bus.read_i2c_block_data(MC_I2C_ADDR,ADC_BAR_ADDR, 2)
            return ((msb << 8) | lsb) / 1e3

    def read_encoders(self):
        with self.i2c_lock:
            encoders = struct.unpack('4L', bytes(self.bus.read_i2c_block_data(MC_I2C_ADDR, MOTOR_ENCODER_TOTAT_ADDR, 16)))
            print(encoders)

    def print(self):
        print("Battery: ", self.read_battery())
        print('Encoders: f', self.read_encoders())


    def __del__(self):
        with self.i2c_lock:
            self.bus.close()
 