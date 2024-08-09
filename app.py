#!/home/nps/tank/bin/python3
import threading
import time
import qwiic_icm20948
import busio
import board
import numpy as np
from scipy.signal import butter, lfilter
from dataclasses import dataclass, field
from filterpy.kalman import KalmanFilter

def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=True)
    return b, a

def butter_highpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=True)
    return b, a

tank_filter = KalmanFilter(dim_x=2, dim_z=1)
print('x = ', tank_filter.x.T)
print('R = ', tank_filter.R)
print('Q = \n', tank_filter.Q)

# Singleton 
class IMU():
    def __init__(self, i2c, lock):
        self.i2c_bus = i2c
        self.i2c_lock = lock
        self.imu_lock = threading.Lock()
        self.polling = 50
        self.running = False
        self.thread = None
        self.lpf = butter_lowpass(10.0, self.polling, order = 5)
        self.hpf = butter_highpass(0.1, self.polling, order = 5)

        # Initialize data buffers (e.g., length of 20 samples)
        self.index = 0
        self.buffer_length = 20

        self.accel_x_buffer = np.zeros(self.buffer_length)
        self.accel_y_buffer = np.zeros(self.buffer_length)
        self.accel_z_buffer = np.zeros(self.buffer_length)
        self.gyro_x_buffer  = np.zeros(self.buffer_length)
        self.gyro_y_buffer  = np.zeros(self.buffer_length)
        self.gyro_z_buffer  = np.zeros(self.buffer_length)
        self.mag_x_buffer   = np.zeros(self.buffer_length)
        self.mag_y_buffer   = np.zeros(self.buffer_length)
        self.mag_z_buffer   = np.zeros(self.buffer_length)

        with self.i2c_lock:
            self.imu = qwiic_icm20948.QwiicIcm20948()
            self.imu.setFullScaleRangeAccel(16)
            if not self.imu.connected:
                raise Exception("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
            self.imu.begin()

    def read_data(self):
        with self.i2c_lock:
            if self.imu.dataReady():
                self.imu.getAgmt()
                with self.imu_lock:
                    i = self.index
                    self.accel_x_buffer[i] =  self.imu.axRaw
                    self.accel_y_buffer[i] =  self.imu.ayRaw
                    self.accel_z_buffer[i] =  self.imu.azRaw
                    self.gyro_x_buffer [i] =  self.imu.gxRaw
                    self.gyro_y_buffer [i] =  self.imu.gyRaw
                    self.gyro_z_buffer [i] =  self.imu.gzRaw
                    self.mag_x_buffer  [i] =  self.imu.mxRaw
                    self.mag_y_buffer  [i] =  self.imu.myRaw
                    self.mag_z_buffer  [i] =  self.imu.mzRaw
                    self.index = (i + 1) % self.buffer_length

    def run(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.read_data())
            self.thread.start()
            self.print_thread = threading.Thread(target=self.print_poller())
            self.print_thread.start()

    def stop(self):
        if self.running:
            self.running = False
            if self.thread:
                self.thread.join()

    def _poll(self):
        while self.running:
            self.read_data()
            time.sleep(1.0/self.polling)

    def _poll_print(self):
        while self.running:
            self.print_data()
            time.sleep(1.0)

    def print_data(self):
        
                    self.accel_x_buffer[i] =  self.imu.axRaw
                    self.accel_y_buffer[i] =  self.imu.ayRaw
                    self.accel_z_buffer[i] =  self.imu.azRaw
                    self.gyro_x_buffer [i] =  self.imu.gxRaw
                    self.gyro_y_buffer [i] =  self.imu.gyRaw
                    self.gyro_z_buffer [i] =  self.imu.gzRaw
                    self.mag_x_buffer  [i] =  self.imu.mxRaw
                    self.mag_y_buffer  [i] =  self.imu.myRaw
                    self.mag_z_buffer  [i] =  self.imu.mzRaw

        print(
            '{: 06d}'.format(agm.acc[0]),
             '\t', '{: 06d}'.format(agm.acc[1]),
             '\t', '{: 06d}'.format(agm.acc[2]),
             '\t', '{: 06d}'.format(agm.gyr[0]),
             '\t', '{: 06d}'.format(agm.gyr[1]),
             '\t', '{: 06d}'.format(agm.gyr[2]),
             '\t', '{: 06d}'.format(agm.mag[0]),
             '\t', '{: 06d}'.format(agm.mag[1]),
             '\t', '{: 06d}'.format(agm.mag[2])
        )


class App():
    def __init__(self):
        self.i2c_lock = threading.Lock()
        self.i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.imu = IMU(App.i2c_bus, App.i2c_lock)

    def run(self):
        while True:
            self.imu.print_data()

app = App()
app.run()