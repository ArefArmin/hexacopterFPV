"""
Hexacopter Configuration File
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 18:10:58 UTC
"""

# System Configuration
SYSTEM_CONFIG = {
    'WATCHDOG_TIMEOUT': 5000  # 5 seconds
}

# Pin Configurations
PINS = {
    # Motor Driver Pins - DRV8833
    'MOTORS': {
        # Front Motors
        'M1': {'IN1': 32, 'IN2': 33},  # Front Right - CW
        'M2': {'IN1': 25, 'IN2': 26},  # Front Left - CCW
        # Middle Motors
        'M3': {'IN1': 27, 'IN2': 14},  # Middle Right - CCW
        'M4': {'IN1': 12, 'IN2': 13},  # Middle Left - CW
        # Back Motors
        'M5': {'IN1': 23, 'IN2': 22},  # Back Right - CW
        'M6': {'IN1': 21, 'IN2': 19}   # Back Left - CCW
    },
    
    # I2C Configuration
    'I2C': {
        'SCL': 22,
        'SDA': 21,
        'FREQUENCY': 400000  # 400kHz
    },
    
    # Camera Pins (OV2640)
    'CAMERA': {
        'VSYNC': 35,
        'HREF': 34,
        'PCLK': 39,
        'XCLK': 36,
        'RESET': 2,
        'D0': 4,
        'D1': 5,
        'D2': 18,
        'D3': 19,
        'D4': 16,
        'D5': 17,
        'D6': 15,
        'D7': 14,
        'SIOD': 21,  # Shared with I2C SDA
        'SIOC': 22   # Shared with I2C SCL
    },
    
    # VL53L0X Distance Sensor
    'VL53L0X': {
        'XSHUT': 17,
        'GPIO1': 5,
        # SCL and SDA shared with I2C bus
    },
    
    # GY-87 (MPU6050 + HMC5883L + BMP180)
    'GY87': {
        'INTA': 4,
        'DRDY': 16,
        # SCL and SDA shared with I2C bus
    },
    
    # Battery Monitoring
    'BATTERY': {
        'VOLTAGE': 36,  # GPIO36 (VP)
    }
}

# I2C Device Addresses
I2C_ADDRESSES = {
    'MPU6050': 0x68,
    'HMC5883L': 0x1E,
    'BMP180': 0x77,
    'VL53L0X': 0x29
}

# Physical Parameters
PHYSICAL_PARAMS = {
    'WEIGHT': 0.103,  # Total weight in kg
    'CENTER_TO_MID_MOTOR': 0.075,  # Distance from center to middle motors in meters
    'CENTER_TO_FRONT_BACK_MOTOR': 0.0832,  # Distance from center to front/back motors in meters
    'INERTIA': {
        'IX': 1.662e+08,  # kg.m²
        'IY': 1.442e+08,  # kg.m²
        'IZ': 2.941e+08   # kg.m²
    }
}

# Motor Configuration
MOTOR_PARAMS = {
    'MAX_THRUST': 0.245,  # Maximum thrust per motor in Newtons
    'VOLTAGE': 3.7,       # Operating voltage
    'KV': 15000,         # Motor KV rating
    'PWM_FREQ': 20000,   # PWM frequency in Hz
    'PWM_RESOLUTION': 10, # 10-bit resolution (0-1023)
    'MIN_THROTTLE': 0,
    'MAX_THROTTLE': 1023
}

# Camera Settings
CAMERA_SETTINGS = {
    'FPV': {
        'RESOLUTION': (320, 240),
        'FPS': 60,
        'QUALITY': 12,
        'FORMAT': 'JPEG'
    },
    'PHOTO': {
        'RESOLUTION': (1600, 1200),
        'QUALITY': 4,
        'FORMAT': 'JPEG'
    },
    'VIDEO': {
        'RESOLUTION': (800, 600),
        'FPS': 30,
        'QUALITY': 8,
        'FORMAT': 'JPEG'
    }
}

# VL53L0X Configuration
VL53L0X_PARAMS = {
    'TIMING_BUDGET': 33000,  # Measurement timing budget in microseconds
    'MIN_SAFE_DISTANCE': 500,  # Minimum safe distance in mm (50cm)
    'CEILING_OFFSET': 10,     # Distance from sensor to drone's top in mm
}

# IMU Configuration
IMU_PARAMS = {
    'SAMPLE_RATE': 200,  # Hz
    'GYRO_RANGE': 2000,  # deg/sec
    'ACCEL_RANGE': 8,    # g
    'MAG_RATE': 75,     # Hz
    'BARO_RATE': 50     # Hz
}

# Flight Control Parameters
FLIGHT_PARAMS = {
    'MAX_ANGLE': 30.0,         # Maximum roll/pitch angle in degrees
    'MAX_YAW_RATE': 90.0,      # Maximum yaw rate in degrees/second
    'MAX_VERTICAL_SPEED': 1.0,  # Maximum vertical speed in m/s
    'MAX_ALTITUDE': 50.0,       # Maximum altitude in meters
    'UPDATE_RATE': 200,         # Control loop frequency in Hz
}

# PID Parameters (tuned for 200Hz update rate)
PID_PARAMS = {
    'altitude': {
        'P': 1.8,
        'I': 0.05,
        'D': 0.25,
        'MAX_I': 100,
        'MAX_OUTPUT': 400
    },
    'roll': {
        'P': 5.2,
        'I': 0.03,
        'D': 0.35,
        'MAX_I': 50,
        'MAX_OUTPUT': 400
    },
    'pitch': {
        'P': 5.0,
        'I': 0.03,
        'D': 0.35,
        'MAX_I': 50,
        'MAX_OUTPUT': 400
    },
    'yaw': {
        'P': 2.5,
        'I': 0.025,
        'D': 0.15,
        'MAX_I': 50,
        'MAX_OUTPUT': 200
    }
}

# Battery Monitoring
BATTERY_PARAMS = {
    'CELLS': 1,
    'MIN_VOLTAGE': 3.3,        # Minimum safe voltage per cell
    'WARNING_VOLTAGE': 3.5,    # Warning voltage per cell
    'MAX_VOLTAGE': 4.2,        # Maximum voltage per cell
    'VOLTAGE_DIVIDER': {
        'R1': 48000,           # 48kΩ
        'R2': 10000            # 10kΩ
    },
    'ADC_REF': 1.1,           # ADC reference voltage
    'ADC_BITS': 12,           # ADC resolution
    'UPDATE_RATE': 1          # Battery status update rate in Hz
}

# Web Server Configuration
WEB_SERVER = {
    'PORT': 80,
    'HOST': '0.0.0.0',
    'WEBSOCKET_PORTS': {
        'telemetry': 8000,
        'video': 8001,
        'control': 8002,
        'system': 8003
    }
}

# WiFi Configuration
WIFI_CONFIG = {
    'SSID': 'Hexacopter_AP',
    'PASSWORD': 'hexacopter123',
    'CHANNEL': 6,
    'MAX_CLIENTS': 1
}

# Camera Settings
CAMERA_SETTINGS = {
    'fpv': {
        'resolution': {
            'width': 320,
            'height': 240,
            'reg': 0x10  # QVGA
        },
        'quality': 12,  # Higher number = lower quality
        'fps': 60
    },
    'photo': {
        'resolution': {
            'width': 1600,
            'height': 1200,
            'reg': 0x00  # UXGA
        },
        'quality': 4    # Best quality
    },
    'video': {
        'resolution': {
            'width': 800,
            'height': 600,
            'reg': 0x08  # SVGA
        },
        'quality': 8,
        'fps': 30
    },
    'buffer_size': 32768  # 32KB buffer for frame data
}