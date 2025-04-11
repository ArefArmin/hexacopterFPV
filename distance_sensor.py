"""
Distance Sensor (VL53L0X) Module for Ceiling Detection and Height Control
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 18:32:33 UTC

This module handles the VL53L0X laser distance sensor for ceiling distance measurement
and throttle control for maintaining safe distance from overhead obstacles.
"""

from machine import Pin, I2C
from micropython import const
import time
from config import PINS, I2C_ADDRESSES, VL53L0X_PARAMS

# VL53L0X Register Addresses
SYSRANGE_START = const(0x00)
RESULT_INTERRUPT_STATUS = const(0x13)
RESULT_RANGE_STATUS = const(0x14)
SYSTEM_INTERRUPT_CLEAR = const(0x0B)

class DistanceSensor:
    def __init__(self):
        """Initialize VL53L0X distance sensor"""
        # Configure I2C
        self.i2c = I2C(0, scl=Pin(PINS['I2C']['SCL']), 
                       sda=Pin(PINS['I2C']['SDA']),
                       freq=PINS['I2C']['FREQUENCY'])
        
        # Configure XSHUT pin
        self.xshut = Pin(PINS['VL53L0X']['XSHUT'], Pin.OUT)
        
        # Reset and initialize sensor
        self.reset()
        self._init_sensor()
        
        # Frame offset (distance between sensor and top frame of hexacopter)
        self.FRAME_OFFSET = 10  # 10mm (1cm) - distance from sensor to top frame
        
        # Minimum safe distance from ceiling (500mm = 50cm)
        self.MIN_SAFE_DISTANCE = 500
        
        # Safety margin before starting to limit throttle (50mm = 5cm)
        self.SAFETY_MARGIN = 50
        
        # Last valid reading
        self.last_valid_distance = None
        self.last_reading_time = 0

    def reset(self):
        """Reset the sensor using XSHUT pin"""
        self.xshut.value(0)
        time.sleep_ms(10)
        self.xshut.value(1)
        time.sleep_ms(10)

    def _init_sensor(self):
        """Initialize the sensor with optimal settings"""
        # Basic initialization
        self._write_reg(0x88, 0x00)
        self._write_reg(0x80, 0x01)
        self._write_reg(0xFF, 0x01)
        self._write_reg(0x00, 0x00)
        
        # Set measurement timing budget to 33ms for better accuracy
        self._write_reg(0x00, 0x01)
        self._write_reg(0xFF, 0x00)
        self._write_reg(0x80, 0x00)

    def _write_reg(self, reg, data):
        """Write data to register"""
        if isinstance(data, list):
            data = bytes(data)
        elif not isinstance(data, bytes):
            data = bytes([data])
        self.i2c.writeto_mem(I2C_ADDRESSES['VL53L0X'], reg, data)

    def _read_reg(self, reg, length=1):
        """Read data from register"""
        return self.i2c.readfrom_mem(I2C_ADDRESSES['VL53L0X'], reg, length)

    def get_raw_distance(self):
        """Get raw distance measurement in millimeters"""
        # Start measurement
        self._write_reg(SYSRANGE_START, 0x01)
        
        # Wait for data ready (max 100ms timeout)
        start_time = time.ticks_ms()
        while (self._read_reg(RESULT_INTERRUPT_STATUS)[0] & 0x07) == 0:
            if time.ticks_diff(time.ticks_ms(), start_time) > 100:
                return None
        
        # Read range data
        data = self._read_reg(RESULT_RANGE_STATUS + 10, 2)
        self._write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01)
        
        # Convert to distance
        distance = (data[0] << 8) | data[1]
        return distance

    def get_ceiling_distance(self):
        """
        Get actual distance to ceiling, accounting for frame offset.
        Returns distance in millimeters or None if invalid reading.
        """
        raw_distance = self.get_raw_distance()
        
        if raw_distance is None or raw_distance < self.FRAME_OFFSET:
            return self.last_valid_distance
        
        # Subtract frame offset to get actual distance to ceiling
        actual_distance = raw_distance - self.FRAME_OFFSET
        
        self.last_valid_distance = actual_distance
        self.last_reading_time = time.ticks_ms()
        
        return actual_distance

    def adjust_throttle(self, requested_throttle):
        """
        Adjust throttle input based on ceiling distance.
        
        Args:
            requested_throttle: Original throttle input from joystick (0.0 to 1.0)
            
        Returns:
            Modified throttle value (0.0 to 1.0) that maintains safe distance
        """
        distance = self.get_ceiling_distance()
        
        # If we can't get a valid reading, return original throttle but slightly reduced
        if distance is None:
            return requested_throttle * 0.95  # 5% safety reduction
            
        # If we're already too close to the ceiling
        if distance <= self.MIN_SAFE_DISTANCE:
            # Only allow downward movement
            if requested_throttle < 0.5:  # Assuming 0.5 is hover throttle
                return requested_throttle
            else:
                return 0.5  # Maintain hover
                
        # If we're approaching the minimum safe distance
        elif distance <= (self.MIN_SAFE_DISTANCE + self.SAFETY_MARGIN):
            # Calculate how close we are to the minimum safe distance
            danger_zone = self.SAFETY_MARGIN
            distance_in_danger = distance - self.MIN_SAFE_DISTANCE
            
            # Calculate throttle limitation factor (0.0 to 1.0)
            limitation_factor = distance_in_danger / danger_zone
            
            # If trying to go up (throttle > 0.5)
            if requested_throttle > 0.5:
                # Gradually limit the throttle as we get closer to MIN_SAFE_DISTANCE
                max_additional_throttle = (requested_throttle - 0.5) * limitation_factor
                return 0.5 + max_additional_throttle
            else:
                # Allow downward movement
                return requested_throttle
                
        # If we're far enough from the ceiling, allow full throttle control
        else:
            return requested_throttle

    def get_sensor_status(self):
        """Get current sensor status"""
        distance = self.get_ceiling_distance()
        
        return {
            'ceiling_distance': distance,  # in mm
            'is_safe': False if distance is None else distance >= self.MIN_SAFE_DISTANCE,
            'last_reading_time': self.last_reading_time,
            'can_go_up': False if distance is None else distance > self.MIN_SAFE_DISTANCE
        }