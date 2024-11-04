import os
import sys

# Path to your virtual environment
venv_path = os.path.join(os.path.dirname(__file__), "tankenv")

# Add the virtual environment's site-packages to the sys.path
site_packages = os.path.join(venv_path, "lib", "python3.x", "site-packages")
sys.path.insert(0, site_packages)

import time
import numpy as np
from icm20948 import ICM20948
from ahrs.filters import Madgwick

# Initialize the ICM-20948 sensor
sensor = ICM20948(i2c_addr=0x69)

# Initialize the Madgwick filter
madgwick = Madgwick()

# Function to read sensor data
def read_sensor():
    accel_gyro_data = sensor.read_accelerometer_gyro_data()
    mag_data = sensor.read_magnetometer_data()
    
    return accel_gyro_data, mag_data

# Main loop
while True:
    accel_gyro, mag = read_sensor()
    
    # Convert sensor data to numpy arrays
    print(f'Accel: {accel_gyro[0]:6.2f}, {accel_gyro[1]:6.2f}, {accel_gyro[2]:6.2f}    Gyro: {accel_gyro[3]:6.2f},{accel_gyro[4]:6.2f},{accel_gyro[5]:6.2f}     Mag: {mag[0]:6.2f}, {mag[1]:6.2f},{mag[2]:6.2f}')
    
    time.sleep(0.1)