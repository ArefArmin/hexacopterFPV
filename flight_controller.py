"""
Flight Controller Module for Hexacopter
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 19:10:42 UTC

This module integrates all components and handles the main flight control logic.
"""

import time
from machine import Timer, Pin
import json
from config import (PINS, MOTOR_PARAMS, FLIGHT_PARAMS, 
                   PID_PARAMS, BATTERY_PARAMS)
from motors import Motors
from imu import IMU
from distance_sensor import DistanceSensor

class FlightController:
    def __init__(self):
        """Initialize flight controller"""
        # Initialize components
        self.motors = Motors()
        self.imu = IMU()
        self.distance_sensor = DistanceSensor()
        
        # Initialize control variables
        self.armed = False
        self.flight_mode = 'STABILIZE'  # STABILIZE, ALTITUDE_HOLD
        self.throttle = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        
        # Initialize PID controllers
        self.pid_roll = PIDController(**PID_PARAMS['roll'])
        self.pid_pitch = PIDController(**PID_PARAMS['pitch'])
        self.pid_yaw = PIDController(**PID_PARAMS['yaw'])
        self.pid_altitude = PIDController(**PID_PARAMS['altitude'])
        
        # System status
        self.last_update = time.ticks_ms()
        self.system_status = {
            'armed': False,
            'flight_mode': 'STABILIZE',
            'battery_voltage': 0.0,
            'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'ceiling_distance': 0,
            'motors': {},
            'errors': []
        }
        
        # Setup control loop timer
        self.control_timer = Timer(0)
        self.control_timer.init(
            period=int(1000/FLIGHT_PARAMS['UPDATE_RATE']),
            mode=Timer.PERIODIC,
            callback=self._control_loop
        )

    def arm(self):
        """Arm the hexacopter if all systems are ready"""
        if not self._pre_arm_checks():
            return False
            
        print("Arming motors...")
        self.armed = True
        self.motors.arm()
        return True
        
    def disarm(self):
        """Disarm the hexacopter"""
        print("Disarming motors...")
        self.armed = False
        self.motors.disarm()
        self.reset_pids()
        
    def _pre_arm_checks(self):
        """Perform pre-arm safety checks"""
        checks = {
            'IMU Ready': self.imu.get_attitude() is not None,
            'Battery OK': self._check_battery_voltage(),
            'Sensors OK': self._check_sensors(),
            'Distance Safe': self.distance_sensor.get_ceiling_distance() > 
                           self.distance_sensor.MIN_SAFE_DISTANCE
        }
        
        failed_checks = [check for check, status in checks.items() if not status]
        
        if failed_checks:
            print(f"Pre-arm checks failed: {', '.join(failed_checks)}")
            return False
        return True
        
    def _check_battery_voltage(self):
        """Check if battery voltage is above minimum"""
        # Implement battery voltage check
        return True  # Placeholder
        
    def _check_sensors(self):
        """Check if all sensors are responding"""
        try:
            self.imu.get_attitude()
            self.distance_sensor.get_ceiling_distance()
            return True
        except:
            return False
            
    def reset_pids(self):
        """Reset all PID controllers"""
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        self.pid_altitude.reset()

    def process_joystick_input(self, left_stick, right_stick):
        """
        Process joystick inputs and update control variables
        left_stick: (x, y) for yaw and throttle
        right_stick: (x, y) for roll and pitch
        """
        # Extract joystick values (-1 to 1)
        yaw = left_stick[0]
        throttle_raw = (left_stick[1] + 1) / 2  # Convert to 0 to 1
        roll = right_stick[0]
        pitch = right_stick[1]
        
        # Apply ceiling distance safety control
        self.throttle = self.distance_sensor.adjust_throttle(throttle_raw)
        
        # Apply exponential curve for smoother control
        self.roll = self._apply_expo(roll, 0.3) * FLIGHT_PARAMS['MAX_ANGLE']
        self.pitch = self._apply_expo(pitch, 0.3) * FLIGHT_PARAMS['MAX_ANGLE']
        self.yaw = self._apply_expo(yaw, 0.3) * FLIGHT_PARAMS['MAX_YAW_RATE']

    def _apply_expo(self, value, expo):
        """Apply exponential curve to value for smoother control"""
        return value * (1 - expo + expo * abs(value))

    def _control_loop(self, timer):
        """Main control loop running at UPDATE_RATE frequency"""
        if not self.armed:
            self.motors.stop_all()
            return
            
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_update) / 1000.0
        
        try:
            # Get sensor data
            attitude = self.imu.get_attitude()
            ceiling_distance = self.distance_sensor.get_ceiling_distance()
            
            # Update PIDs
            roll_correction = self.pid_roll.update(self.roll, attitude['roll'], dt)
            pitch_correction = self.pid_pitch.update(self.pitch, attitude['pitch'], dt)
            yaw_correction = self.pid_yaw.update(self.yaw, attitude['gyro']['z'], dt)
            
            # Calculate motor outputs
            motor_outputs = self._mix_outputs(
                self.throttle,
                roll_correction,
                pitch_correction,
                yaw_correction
            )
            
            # Apply motor outputs
            self.motors.set_motor_speeds(*motor_outputs)
            
            # Update system status
            self._update_system_status(attitude, ceiling_distance)
            
        except Exception as e:
            print(f"Control loop error: {str(e)}")
            self.system_status['errors'].append(str(e))
            self.emergency_land()
            
        self.last_update = current_time

    def _mix_outputs(self, throttle, roll, pitch, yaw):
        """Mix control inputs into motor outputs"""
        # Convert throttle from 0-1 to PWM range
        throttle_pwm = int(throttle * MOTOR_PARAMS['MAX_THROTTLE'])
        
        # Apply mixing matrix from motors.py
        return self.motors.set_motor_speeds(throttle_pwm, pitch, roll, yaw)

    def _update_system_status(self, attitude, ceiling_distance):
        """Update system status dictionary"""
        self.system_status.update({
            'armed': self.armed,
            'flight_mode': self.flight_mode,
            'attitude': attitude,
            'ceiling_distance': ceiling_distance,
            'motors': self.motors.get_motor_speeds()
        })

    def emergency_land(self):
        """Perform emergency landing procedure"""
        print("EMERGENCY LANDING INITIATED")
        
        # Gradually reduce throttle while maintaining stability
        while self.throttle > 0:
            self.throttle = max(0, self.throttle - 0.02)
            time.sleep_ms(50)
            
        self.disarm()

    def get_telemetry(self):
        """Get current telemetry data"""
        return json.dumps(self.system_status)

class PIDController:
    def __init__(self, P, I, D, MAX_I, MAX_OUTPUT):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.max_i = MAX_I
        self.max_output = MAX_OUTPUT
        self.reset()
        
    def reset(self):
        self.integral = 0
        self.last_error = 0
        
    def update(self, setpoint, actual, dt):
        error = setpoint - actual
        
        # Proportional term
        P = error * self.Kp
        
        # Integral term
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_i), -self.max_i)
        I = self.integral * self.Ki
        
        # Derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        D = derivative * self.Kd
        
        # Calculate total output
        output = P + I + D
        
        # Limit output
        output = max(min(output, self.max_output), -self.max_output)
        
        self.last_error = error
        return output