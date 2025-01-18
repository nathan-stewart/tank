# filepath: /home/nps/src/tank2/src/motor_driver/motor_driver.py
import smbus2 as smbus
import threading
import struct

MC_I2C_ADDR = 0x34
ADC_BAR_ADDR = 0x00
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_SPEED_ADDR = 0x33
MOTOR_ENCODER_TOTAT_ADDR = 0x3c
MOTOR_TYPE_JGB37_520_12V_110RPM = 3

class MotorDriver:
    def __init__(self):
        self.i2c_lock = threading.Lock()
        self.bus = smbus.SMBus(1)

        with self.i2c_lock:
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM)
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, 0)
            self.bus.write_byte_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, 0)

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
            self.bus.write_i2c_block_data(MC_I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, state[0], state[1], turret[0], turret[1])

    def read_battery(self):
        with self.i2c_lock:
            lsb, msb = self.bus.read_i2c_block_data(MC_I2C_ADDR, ADC_BAR_ADDR, 2)
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