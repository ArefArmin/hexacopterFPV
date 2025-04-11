"""
Motor Control Module for Hexacopter
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 18:16:37 UTC

This module handles the control of six brushless motors using DRV8833 drivers.
Motor arrangement:
    M1(CW)    M2(CCW)
       \        /
        \      /
    M3(CCW)----M4(CW)
        /      \
       /        \
    M5(CW)    M6(CCW)
"""

from machine import Pin, PWM
from micropython import const
from config import PINS, MOTOR_PARAMS
import time

class Motors:
    def __init__(self):
        """Initialize all motor control pins and PWM channels"""
        # Motor objects dictionary
        self.motors = {}
        
        # Initialize each motor with its corresponding pins
        for motor_id, pins in PINS['MOTORS'].items():
            self.motors[motor_id] = {
                'pwm1': PWM(Pin(pins['IN1']), 
                           freq=MOTOR_PARAMS['PWM_FREQ'],
                           duty=0),
                'pwm2': PWM(Pin(pins['IN2']),
                           freq=MOTOR_PARAMS['PWM_FREQ'],
                           duty=0),
                'current_speed': 0
            }
        
        self.armed = False
        self.emergency_stop = False
        
        # Motor mixing matrix for different movements
        # [M1, M2, M3, M4, M5, M6]
        self.mix_matrix = {
            'throttle': [ 1.0,  1.0,  1.0,  1.0,  1.0,  1.0], # Vertical thrust
            'pitch':    [ 1.0, -1.0,  0.0,  0.0, -1.0,  1.0], # Forward/Backward
            'roll':     [-1.0, -1.0,  1.0, -1.0,  1.0,  1.0], # Left/Right
            'yaw':      [ 1.0, -1.0,  1.0, -1.0,  1.0, -1.0]  # Rotation (Updated)
        }
        
        # Initialize all motors to zero
        self.stop_all()

    def arm(self):
        """Arm the motors with safety checks"""
        if not self.emergency_stop:
            print("Arming motors...")
            self.armed = True
            # Set all motors to minimum throttle
            for motor in self.motors.values():
                self._set_motor_speed(motor, MOTOR_PARAMS['MIN_THROTTLE'])
            time.sleep(0.5)
            return True
        return False

    def disarm(self):
        """Disarm all motors"""
        print("Disarming motors...")
        self.armed = False
        self.stop_all()

    def emergency_stop_all(self):
        """Emergency stop all motors immediately"""
        print("EMERGENCY STOP!")
        self.emergency_stop = True
        self.armed = False
        self.stop_all()

    def reset_emergency(self):
        """Reset emergency stop state"""
        self.emergency_stop = False

    def stop_all(self):
        """Stop all motors"""
        for motor in self.motors.values():
            self._set_motor_speed(motor, 0)

    def _set_motor_speed(self, motor, speed):
        """
        Set the speed of a single motor using the DRV8833 driver
        speed: 0 to 1023 (10-bit PWM)
        """
        if speed > 0:
            motor['pwm1'].duty(speed)
            motor['pwm2'].duty(0)
        else:
            motor['pwm1'].duty(0)
            motor['pwm2'].duty(0)
        motor['current_speed'] = speed

    def set_motor_speeds(self, throttle, pitch, roll, yaw):
        """
        Set motor speeds based on control inputs
        All inputs should be normalized (-1 to 1)
        """
        if not self.armed or self.emergency_stop:
            return

        # Convert throttle from 0-1 to PWM range
        throttle_pwm = int(throttle * MOTOR_PARAMS['MAX_THROTTLE'])
        
        # Calculate motor speeds using mixing matrix
        speeds = [0] * 6
        for i in range(6):
            speed = (
                throttle_pwm * self.mix_matrix['throttle'][i] +
                pitch * self.mix_matrix['pitch'][i] * MOTOR_PARAMS['MAX_THROTTLE'] * 0.5 +
                roll * self.mix_matrix['roll'][i] * MOTOR_PARAMS['MAX_THROTTLE'] * 0.5 +
                yaw * self.mix_matrix['yaw'][i] * MOTOR_PARAMS['MAX_THROTTLE'] * 0.3
            )
            
            # Constrain speed to valid PWM range
            speeds[i] = max(min(int(speed), MOTOR_PARAMS['MAX_THROTTLE']), 
                          MOTOR_PARAMS['MIN_THROTTLE'])

        # Apply speeds to motors
        motor_names = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6']
        for motor_name, speed in zip(motor_names, speeds):
            self._set_motor_speed(self.motors[motor_name], speed)

    def get_motor_speeds(self):
        """Return current speeds of all motors"""
        return {motor_id: motor['current_speed'] 
                for motor_id, motor in self.motors.items()}

    def test_motors(self):
        """Test each motor individually"""
        if not self.armed:
            print("Motors must be armed first!")
            return False
            
        test_speed = MOTOR_PARAMS['MAX_THROTTLE'] // 4  # 25% power for testing
        
        for motor_id, motor in self.motors.items():
            print(f"Testing {motor_id}...")
            self._set_motor_speed(motor, test_speed)
            time.sleep(1)
            self._set_motor_speed(motor, 0)
            time.sleep(1)
        
        print("Motor test complete")
        return True

    def calibrate_escs(self):
        """Calibrate the ESCs (if needed)"""
        print("DRV8833 drivers don't require ESC calibration")
        print("Skipping calibration step...")
        return True

    def set_all_throttle(self, throttle):
        """Set all motors to the same throttle value"""
        if not self.armed or self.emergency_stop:
            return False
            
        if 0 <= throttle <= 1:
            pwm_value = int(throttle * MOTOR_PARAMS['MAX_THROTTLE'])
            for motor in self.motors.values():
                self._set_motor_speed(motor, pwm_value)
            return True
        return False