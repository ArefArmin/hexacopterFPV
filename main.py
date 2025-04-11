"""
Main Application Entry for Hexacopter Control System
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 21:07:11 UTC

This module initializes and coordinates all subsystems of the hexacopter.
"""

import machine
import time
import json
import gc
from config import SYSTEM_CONFIG, WIFI_CONFIG, MOTOR_PARAMS, FLIGHT_PARAMS  # اضافه کردن تمام تنظیمات مورد نیاز
from motors import Motors
from imu import IMU
from distance_sensor import DistanceSensor
from camera import OV2640
from flight_controller import FlightController
from web_interface import WebInterface

def init_system():
    """Initialize system components"""
    print("\nHexacopter Control System")
    print("Author: ArefArmin")
    print("Version: 1.0.0")
    print("Starting system initialization...")
    
    # Setup watchdog timer with timeout from config
    wdt = machine.WDT(timeout=SYSTEM_CONFIG['WATCHDOG_TIMEOUT'])
    
    try:
        # Initialize components with status checking
        print("\nInitializing Motors...")
        motors = Motors()
        wdt.feed()
        
        print("Initializing IMU...")
        imu = IMU()
        wdt.feed()
        
        print("Initializing Distance Sensor...")
        distance_sensor = DistanceSensor()
        wdt.feed()
        
        print("Initializing Camera...")
        camera = OV2640()
        wdt.feed()
        
        print("\nInitializing Flight Controller...")
        flight_controller = FlightController(
            motors=motors,
            imu=imu,
            distance_sensor=distance_sensor
        )
        print("Flight Controller initialized successfully")
        wdt.feed()
        
        print("\nInitializing Web Interface...")
        web_interface = WebInterface(
            flight_controller=flight_controller
        )
        print("Web Interface initialized successfully")
        wdt.feed()
        
        return flight_controller, web_interface, wdt
        
    except Exception as e:
        print(f"\nSystem initialization failed: {e}")
        machine.reset()

def main():
    """Main application entry point"""
    gc.enable()
    flight_controller = None
    web_interface = None
    
    try:
        # Initialize system
        flight_controller, web_interface, wdt = init_system()
        print("\nSystem initialization complete!")
        print(f"System ready at: 2025-04-08 21:07:11")
        print(f"User: ArefArmin")
        
        # Start web interface
        web_interface.start()
        
        # Main loop
        while True:
            gc.collect()
            wdt.feed()
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutdown requested...")
        
    except Exception as e:
        print(f"\nCritical error: {e}")
        
    finally:
        # Cleanup
        if web_interface:
            web_interface.stop()
        if flight_controller:
            flight_controller.emergency_land()
        print("\nSystem shutdown complete")

if __name__ == "__main__":
    main()