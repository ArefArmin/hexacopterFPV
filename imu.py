"""
IMU (GY-87) Control Module
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 18:13:52 UTC

This module handles the GY-87 IMU sensor which includes:
- MPU6050 (Gyroscope + Accelerometer)
- HMC5883L (Magnetometer)
- BMP180 (Barometer)
"""

from machine import I2C, Pin
from micropython import const
import time
import math
from config import PINS, I2C_ADDRESSES, IMU_PARAMS

# MPU6050 Registers
MPU6050_PWR_MGMT_1 = const(0x6B)
MPU6050_GYRO_CONFIG = const(0x1B)
MPU6050_ACCEL_CONFIG = const(0x1C)
MPU6050_INT_ENABLE = const(0x38)
MPU6050_ACCEL_XOUT_H = const(0x3B)
MPU6050_GYRO_XOUT_H = const(0x43)
MPU6050_USER_CTRL = const(0x6A)
MPU6050_INT_PIN_CFG = const(0x37)

# HMC5883L Registers
HMC5883L_CONFIG_A = const(0x00)
HMC5883L_CONFIG_B = const(0x01)
HMC5883L_MODE = const(0x02)
HMC5883L_DATA_OUT_X_H = const(0x03)

# BMP180 Registers
BMP180_CONTROL = const(0xF4)
BMP180_CAL_AC1 = const(0xAA)
BMP180_DATA = const(0xF6)

class IMU:
    def __init__(self):
        """Initialize IMU sensors"""
        # Setup I2C
        self.i2c = I2C(0, scl=Pin(PINS['I2C']['SCL']), 
                       sda=Pin(PINS['I2C']['SDA']),
                       freq=PINS['I2C']['FREQUENCY'])
        
        # Initialize interrupt pins
        self.inta_pin = Pin(PINS['GY87']['INTA'], Pin.IN)
        self.drdy_pin = Pin(PINS['GY87']['DRDY'], Pin.IN)
        
        # Sensor calibration values
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}
        self.mag_calibration = {'x': 0, 'y': 0, 'z': 0}
        self.bmp180_cal = {}
        
        # Initialize sensors
        self._init_mpu6050()
        self._init_hmc5883l()
        self._init_bmp180()
        
        # Calibrate sensors
        self.calibrate_sensors()

    def _init_mpu6050(self):
        """Initialize MPU6050"""
        # Wake up MPU6050
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_PWR_MGMT_1, bytes([0x00]))
        time.sleep_ms(100)
        
        # Configure gyroscope range (±2000 deg/s)
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_GYRO_CONFIG, 
                            bytes([0x18]))  # 0x18 = 0b11000 (±2000°/s)
        
        # Configure accelerometer range (±8g)
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_ACCEL_CONFIG,
                            bytes([0x10]))  # 0x10 = 0b10000 (±8g)
        
        # Enable data ready interrupt
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_INT_ENABLE,
                            bytes([0x01]))
        
        # Configure I2C bypass enable for HMC5883L access
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_USER_CTRL,
                            bytes([0x00]))
        self.i2c.writeto_mem(I2C_ADDRESSES['MPU6050'], MPU6050_INT_PIN_CFG,
                            bytes([0x02]))

    def _init_hmc5883l(self):
        """Initialize HMC5883L magnetometer"""
        # Set to continuous measurement mode
        self.i2c.writeto_mem(I2C_ADDRESSES['HMC5883L'], HMC5883L_CONFIG_A,
                            bytes([0x70]))  # 8-average, 75 Hz
        self.i2c.writeto_mem(I2C_ADDRESSES['HMC5883L'], HMC5883L_CONFIG_B,
                            bytes([0x20]))  # Range ±1.3 Ga
        self.i2c.writeto_mem(I2C_ADDRESSES['HMC5883L'], HMC5883L_MODE,
                            bytes([0x00]))  # Continuous measurement

    def _init_bmp180(self):
        """Initialize BMP180 and read calibration data"""
        # Read calibration data
        cal_data = bytearray(22)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['BMP180'], BMP180_CAL_AC1, cal_data)
        
        # Convert calibration data
        self.bmp180_cal = {
            'AC1': (cal_data[0] << 8) + cal_data[1],
            'AC2': (cal_data[2] << 8) + cal_data[3],
            'AC3': (cal_data[4] << 8) + cal_data[5],
            'AC4': (cal_data[6] << 8) + cal_data[7],
            'AC5': (cal_data[8] << 8) + cal_data[9],
            'AC6': (cal_data[10] << 8) + cal_data[11],
            'B1': (cal_data[12] << 8) + cal_data[13],
            'B2': (cal_data[14] << 8) + cal_data[15],
            'MB': (cal_data[16] << 8) + cal_data[17],
            'MC': (cal_data[18] << 8) + cal_data[19],
            'MD': (cal_data[20] << 8) + cal_data[21]
        }

    def calibrate_sensors(self):
        """Calibrate gyroscope and magnetometer"""
        print("Calibrating sensors... Keep the device still!")
        
        # Calibrate gyroscope
        samples = 1000
        sum_x = sum_y = sum_z = 0
        
        for _ in range(samples):
            gyro = self.read_gyroscope_raw()
            sum_x += gyro['x']
            sum_y += gyro['y']
            sum_z += gyro['z']
            time.sleep_ms(2)
        
        self.gyro_offset = {
            'x': sum_x / samples,
            'y': sum_y / samples,
            'z': sum_z / samples
        }
        
        print("Calibration complete!")

    def read_accelerometer(self):
        """Read accelerometer data in g"""
        data = bytearray(6)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['MPU6050'], MPU6050_ACCEL_XOUT_H, data)
        
        accel = {
            'x': (data[0] << 8 | data[1]) / 4096.0,  # ±8g range
            'y': (data[2] << 8 | data[3]) / 4096.0,
            'z': (data[4] << 8 | data[5]) / 4096.0
        }
        return accel

    def read_gyroscope_raw(self):
        """Read raw gyroscope data"""
        data = bytearray(6)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['MPU6050'], MPU6050_GYRO_XOUT_H, data)
        
        return {
            'x': data[0] << 8 | data[1],
            'y': data[2] << 8 | data[3],
            'z': data[4] << 8 | data[5]
        }

    def read_gyroscope(self):
        """Read calibrated gyroscope data in degrees/s"""
        raw = self.read_gyroscope_raw()
        
        return {
            'x': (raw['x'] - self.gyro_offset['x']) / 16.4,  # ±2000°/s range
            'y': (raw['y'] - self.gyro_offset['y']) / 16.4,
            'z': (raw['z'] - self.gyro_offset['z']) / 16.4
        }

    def read_magnetometer(self):
        """Read magnetometer data in Gauss"""
        data = bytearray(6)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['HMC5883L'], HMC5883L_DATA_OUT_X_H, data)
        
        mag = {
            'x': (data[0] << 8 | data[1]) / 1090.0,  # ±1.3 Ga range
            'y': (data[2] << 8 | data[3]) / 1090.0,
            'z': (data[4] << 8 | data[5]) / 1090.0
        }
        return mag

    def read_temperature_pressure(self):
        """Read temperature (°C) and pressure (Pa) from BMP180"""
        # Start temperature measurement
        self.i2c.writeto_mem(I2C_ADDRESSES['BMP180'], BMP180_CONTROL, bytes([0x2E]))
        time.sleep_ms(5)
        
        # Read temperature
        data = bytearray(2)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['BMP180'], BMP180_DATA, data)
        UT = (data[0] << 8) + data[1]
        
        # Start pressure measurement (oversampling setting 3)
        self.i2c.writeto_mem(I2C_ADDRESSES['BMP180'], BMP180_CONTROL, bytes([0xF4]))
        time.sleep_ms(26)
        
        # Read pressure
        data = bytearray(3)
        self.i2c.readfrom_mem_into(I2C_ADDRESSES['BMP180'], BMP180_DATA, data)
        UP = ((data[0] << 16) + (data[1] << 8) + data[2]) >> (8 - 3)
        
        # Calculate true temperature
        X1 = (UT - self.bmp180_cal['AC6']) * self.bmp180_cal['AC5'] / 32768
        X2 = self.bmp180_cal['MC'] * 2048 / (X1 + self.bmp180_cal['MD'])
        B5 = X1 + X2
        temperature = (B5 + 8) / 16 / 10.0
        
        # Calculate true pressure
        B6 = B5 - 4000
        X1 = (self.bmp180_cal['B2'] * (B6 * B6 / 4096)) / 2048
        X2 = self.bmp180_cal['AC2'] * B6 / 2048
        X3 = X1 + X2
        B3 = (((self.bmp180_cal['AC1'] * 4 + X3) << 3) + 2) / 4
        X1 = self.bmp180_cal['AC3'] * B6 / 8192
        X2 = (self.bmp180_cal['B1'] * (B6 * B6 / 4096)) / 65536
        X3 = ((X1 + X2) + 2) / 4
        B4 = self.bmp180_cal['AC4'] * (X3 + 32768) / 32768
        B7 = (UP - B3) * (50000 >> 3)
        
        if B7 < 0x80000000:
            pressure = (B7 * 2) / B4
        else:
            pressure = (B7 / B4) * 2
            
        X1 = (pressure / 256) * (pressure / 256)
        X1 = (X1 * 3038) / 65536
        X2 = (-7357 * pressure) / 65536
        pressure = pressure + (X1 + X2 + 3791) / 16
        
        return temperature, pressure

    def get_attitude(self):
        """Calculate roll, pitch, and yaw angles in degrees"""
        accel = self.read_accelerometer()
        gyro = self.read_gyroscope()
        mag = self.read_magnetometer()
        
        # Calculate roll and pitch from accelerometer
        roll = math.atan2(accel['y'], accel['z']) * 180/math.pi
        pitch = math.atan2(-accel['x'], 
                          math.sqrt(accel['y']*accel['y'] + accel['z']*accel['z'])) * 180/math.pi
        
        # Calculate yaw from magnetometer with tilt compensation
        cos_roll = math.cos(roll * math.pi/180)
        sin_roll = math.sin(roll * math.pi/180)
        cos_pitch = math.cos(pitch * math.pi/180)
        sin_pitch = math.sin(pitch * math.pi/180)
        
        Xh = (mag['x'] * cos_pitch + mag['z'] * sin_pitch)
        Yh = (mag['x'] * sin_roll * sin_pitch + mag['y'] * cos_roll - 
              mag['z'] * sin_roll * cos_pitch)
        
        yaw = math.atan2(Yh, Xh) * 180/math.pi
        if yaw < 0:
            yaw += 360
            
        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'gyro': gyro
        }

    def get_altitude(self, ground_pressure=101325):
        """Calculate altitude in meters from pressure"""
        _, pressure = self.read_temperature_pressure()
        return 44330 * (1 - (pressure/ground_pressure)**(1/5.255))