"""
Boot Configuration for Hexacopter
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-09 08:56:03 UTC

This script runs on boot and configures initial system settings.
"""

import gc
import machine

def init_system():
    """Initialize system settings"""
    # Configure CPU frequency
    machine.freq(240000000)  # Set to 240MHz for maximum performance

    # Configure garbage collection
    gc.threshold(100000)
    gc.enable()

    # Configure watchdog
    wdt = machine.WDT(timeout=5000)  # 5 second timeout

    # Configure system LED (optional)
    try:
        led = machine.Pin(2, machine.Pin.OUT)  # Built-in LED on most ESP32 boards
        led.on()  # Turn on LED to indicate boot process
    except:
        pass

    return wdt

def print_system_info():
    """Print system information"""
    print("\nHexacopter System Boot")
    print("----------------------")
    print(f"Boot Time (UTC): 2025-04-09 08:56:03")
    print(f"User: ArefArmin")
    print(f"CPU Frequency: {machine.freq() / 1000000}MHz")
    print(f"Free Memory: {gc.mem_free()} bytes")
    print("----------------------\n")

# Initialize system
try:
    wdt = init_system()
    print_system_info()
    wdt.feed()
except Exception as e:
    print(f"Boot error: {e}")
    machine.reset()