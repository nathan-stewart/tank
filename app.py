#!/usr/bin//python3
import subprocess
import zmq
from motor_driver import MotorDriver
import threading
import time
#import qwiic_icm20948
import smbus2 as smbus
import numpy as np
#from PID_Py.PID import PID
#from filterpy.kalman import KalmanFilter

import subprocess
import atexit

# Server (e.g., on the robot)
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")


# start/stop camera with app
subprocess.call('/usr/bin/sudo /usr/bin/systemctl start camera'.split())
def stop_mediamtx():
    subprocess.call('/usr/bin/sudo /usr/bin/systemctl start camera'.split())
atexit.register(stop_mediamtx)


ACCEL_SENSITIVITY_16G = 16.0 / 32767  # Sensitivity factor for ±16g
GYRO_SENSITIVITY_250DPS = 250.0 / 32767  # Sensitivity factor for ±250dps
MAG_SENSITIVITY_4900UT = 4900.0 / 8192  # Sensitivity factor for ±4900uT

i2c_bus = smbus.SMBus(1)
i2c_lock = threading.Lock()
imu_lock = threading.Lock()
kf_lock  = threading.Lock()

# Initialization
#pid_left  = PID(kp = 2.0, ki = 5.0, kd = 0.0, cycleTime = 0.01)
#pid_right = PID(kp = 2.0, ki = 5.0, kd = 0.0, cycleTime = 0.01)
#pid_left.setPoint = 0
#pid_right.setPoint = 0

#kf = KalmanFilter(dim_x=9, dim_z=9)
# with kf_lock:
#     kf.x = np.zeros(9)  # Initial state
#     kf.P = np.eye(9)  # Initial state covariance
#     kf.F = np.eye(9)  # State transition matrix
#     kf.H = np.eye(9)  # Measurement function
#     kf.Q = np.eye(9)  # Process noise covariance
#     kf.R = np.eye(9)  # Measurement noise covariance
#     z = np.array([0] * 9)  # Measurement vector
#     kf.predict()
#     kf.update(z)


# Singleton
class IMU():
    def __init__(self):
        global kf
        global imu_lock
        global i2c_lock
        global i2c_bus

        self.polling = 50
        self.running = False
        self.threads = []

        self.accel = [0] * 3
        self.gyro = [0] * 3
        self.mag = [0] * 3

        with i2c_lock and imu_lock:
            self.icm20948 = qwiic_icm20948.QwiicIcm20948()
            self.icm20948.begin()
            self.icm20948.setFullScaleRangeAccel(qwiic_icm20948.gpm16)
            self.icm20948.setFullScaleRangeGyro(qwiic_icm20948.dps250)
            self.icm20948.setDLPFcfgAccel(qwiic_icm20948.acc_d11bw5_n17bw)
            self.icm20948.setDLPFcfgGyro(qwiic_icm20948.gyr_d11bw6_n17bw8)
            if not self.icm20948.connected:
                raise Exception("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")

    def __del__(self):
        self.stop()

    def read_data(self):
        global i2c_lock
        global imu_lock
        minval = 16.0
        maxval = 0.0

        ready = False
        with i2c_lock:
            ready = self.icm20948.dataReady()
            if ready:
                self.icm20948.getAgmt()
        if ready:
            with imu_lock:
                self.accel = np.array([self.icm20948.axRaw, self.icm20948.ayRaw, self.icm20948.azRaw]) * ACCEL_SENSITIVITY_16G
                self.gyro = np.array([self.icm20948.gxRaw, self.icm20948.gyRaw, self.icm20948.gzRaw]) * GYRO_SENSITIVITY_250DPS
                rawmag = np.array([self.icm20948.mxRaw, self.icm20948.myRaw, self.icm20948.mzRaw])
                self.mag = np.array(rawmag) / np.linalg.norm(rawmag) # Normalize magnetometer data
                minval = min(minval, np.linalg.norm(self.accel))
                maxval = max(maxval, np.linalg.norm(self.accel))
                if maxval > 4.0 or minval < 0.1:
                    self.print_data()


    def stop(self):
        if self.running:
            self.running = False
            while self.threads:

                self.threads.pop().join()

    def print_data(self):
        print(' '.join(['{: .4f}'.format(value) for value in self.accel])  \
            +  ':(%.4f)' % np.linalg.norm(self.accel) \
            + ' '.join(['{: .4f}'.format(value) for value in self.gyro]) + '   ' \
            + ' '.join(['{: .4f}'.format(value) for value in self.mag]))

    def _poll_data(self):
        global kf
        last_time = time.time()
        while self.running:
            with kf_lock:
                self.read_data()
                current_time = time.time()
                delta_t = current_time - last_time
                last_time = current_time
                kf.F = [[1, delta_t, 0, 0, 0, 0, 0.5 * delta_t ** 2, 0, 0],
                        [0, 1, 0, 0, 0, 0, delta_t, 0, 0],
                        [0, 0, 1, delta_t, 0, 0, 0, 0.5 * delta_t ** 2, 0],
                        [0, 0, 0, 1, 0, 0, 0, delta_t, 0],
                        [0, 0, 0, 0, 1, delta_t, 0, 0, 0.5 * delta_t ** 2],
                        [0, 0, 0, 0, 0, 1, 0, 0, delta_t],
                        [0, 0, 0, 0, 0, 0, 1, delta_t, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 1]]
                with imu_lock:
                    z = np.concatenate([self.accel, self.gyro, self.mag])
                    #print(z)
                    #kf.predict()
                    #kf.update(z)
            time.sleep(1.0/self.polling)

    def run(self):
        if not self.running:
            self.running = True
            self.threads.append(threading.Thread(target=self._poll_data))
            self.threads[-1].start()

#imu = IMU()
#imu.run()
# pid = PID(1.0, 0.1, 0.05, setpoint=0)
# pid.sample_time = 0.01
# pid.output_limits = (-1, 1)
# pid.tunings = (1.0, 0.1, 0.05)
# pid.setpoint = 0
# pid.auto_mode = True
# pid.proportional_on_measurement = False

# PID loop should probably be on a timer
    # current_value1 = measure_process_1()
    # current_value2 = measure_process_2()

    # # Update both PID controllers
    # pid1.update(current_value1)
    # pid2.update(current_value2)

    # # Retrieve the control outputs
    # control_output1 = pid1.output
    # control_output2 = pid2.output

    # # Apply the control outputs to your system
    # apply_control_1(control_output1)
    # apply_control_2(control_output2)

motor = MotorDriver(i2c_lock, i2c_bus)
motor.set_tracks([0, 0])

running = True
while running:
    message = socket.recv_string()
    socket.send_string("Ack")
    event,data = message.split(":")
    if event == "drive":
        motor_left, motor_right = [int(x) for x in data.split(",")]
        motor.set_tracks([motor_left, motor_right])

    time.sleep(0.1)

motor.stop()