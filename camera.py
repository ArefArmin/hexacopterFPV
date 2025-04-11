"""
Camera Control Module (OV2640)
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 19:35:42 UTC

This module handles the OV2640 camera for:
- FPV streaming (60fps, low resolution)
- High-quality photos
- Video recording (30fps, medium resolution)
"""

from machine import Pin, SPI, I2C
import time
import os
from config import PINS, CAMERA_SETTINGS

class OV2640:
    def __init__(self):
        """Initialize OV2640 camera"""
        # Setup I2C for SCCB
        self.i2c = I2C(1, 
                      scl=Pin(PINS['CAMERA']['SCL']), 
                      sda=Pin(PINS['CAMERA']['SDA']), 
                      freq=100000)
                      
        # Setup SPI for data transfer
        self.spi = SPI(2,
                      baudrate=40000000,
                      polarity=0,
                      phase=0,
                      bits=8,
                      firstbit=SPI.MSB,
                      sck=Pin(PINS['CAMERA']['SCK']),
                      mosi=Pin(PINS['CAMERA']['MOSI']),
                      miso=Pin(PINS['CAMERA']['MISO']))
                      
        # Setup control pins
        self.reset_pin = Pin(PINS['CAMERA']['RESET'], Pin.OUT)
        self.xclk_pin = Pin(PINS['CAMERA']['XCLK'], Pin.OUT)
        self.vsync_pin = Pin(PINS['CAMERA']['VSYNC'], Pin.IN)
        self.href_pin = Pin(PINS['CAMERA']['HREF'], Pin.IN)
        self.pclk_pin = Pin(PINS['CAMERA']['PCLK'], Pin.IN)
        
        # Initialize camera
        self.reset()
        self.init_camera()
        
        # Current settings
        self.current_mode = None
        self.recording = False
        self.record_start_time = 0
        self.frame_buffer = bytearray(CAMERA_SETTINGS['buffer_size'])
        
        # Create recordings directory if not exists
        try:
            os.mkdir('/recordings')
        except OSError:
            pass

    def reset(self):
        """Reset the camera"""
        self.reset_pin.value(0)
        time.sleep_ms(10)
        self.reset_pin.value(1)
        time.sleep_ms(10)

    def write_reg(self, reg, val):
        """Write to camera register"""
        self.i2c.writeto_mem(0x30, reg, bytes([val]))
        
    def read_reg(self, reg):
        """Read from camera register"""
        return self.i2c.readfrom_mem(0x30, reg, 1)[0]

    def init_camera(self):
        """Initialize camera with default settings"""
        # Reset all registers
        self.write_reg(0xFF, 0x01)
        self.write_reg(0x12, 0x80)
        time.sleep_ms(100)
        
        # Setup default configuration
        self._set_default_config()
        
        # Set initial mode to FPV
        self.set_mode('fpv')

    def _set_default_config(self):
        """Set default camera configuration"""
        # Basic settings
        self.write_reg(0xFF, 0x00)
        self.write_reg(0x2C, 0xFF)  # Enable all output
        self.write_reg(0x2E, 0xDF)  # Enable all clock
        
        # Color settings
        self.write_reg(0xFF, 0x00)
        self.write_reg(0xC7, 0x10)  # AWB on
        self.write_reg(0x55, 0x40)  # Brightness
        self.write_reg(0x56, 0x40)  # Contrast
        self.write_reg(0x57, 0x40)  # UV saturation
        self.write_reg(0x58, 0x40)  # UV saturation
        
        # Output format: JPEG
        self.write_reg(0xFF, 0x01)
        self.write_reg(0x11, 0x01)  # JPEG output
        self.write_reg(0x12, 0x00)  # Normal mode
        
        # Clock settings for 60fps
        self.write_reg(0x11, 0x01)
        self.write_reg(0x0D, 0x41)

    def set_mode(self, mode):
        """
        Set camera mode
        modes: 'fpv', 'photo', 'video'
        """
        if mode == self.current_mode:
            return
            
        settings = CAMERA_SETTINGS[mode]
        
        # Set resolution
        self.write_reg(0xFF, 0x00)
        self.write_reg(0x0E, settings['resolution']['reg'])
        
        # Set quality/compression
        self.write_reg(0xFF, 0x01)
        self.write_reg(0x44, settings['quality'])
        
        # Set frame rate
        if 'fps' in settings:
            div = int(60 / settings['fps'])
            self.write_reg(0xFF, 0x00)
            self.write_reg(0x0D, 0x40 | div)
            
        self.current_mode = mode

    def capture_frame(self):
        """Capture single frame"""
        # Wait for VSYNC
        while self.vsync_pin.value():
            pass
        while not self.vsync_pin.value():
            pass
            
        # Read frame data
        frame_len = 0
        while frame_len < len(self.frame_buffer):
            if self.href_pin.value():
                while self.pclk_pin.value():
                    pass
                self.frame_buffer[frame_len] = self.spi.read(1)[0]
                frame_len += 1
                while not self.pclk_pin.value():
                    pass
                    
        return self.frame_buffer[:frame_len]

    def start_recording(self):
        """Start video recording"""
        if self.recording:
            return False
            
        self.set_mode('video')
        self.recording = True
        self.record_start_time = time.ticks_ms()
        
        # Generate filename with timestamp
        timestamp = time.localtime()
        self.current_video_file = '/recordings/VID_{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}.mjpeg'.format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        
        return True

    def stop_recording(self):
        """Stop video recording"""
        if not self.recording:
            return False
            
        self.recording = False
        return self.current_video_file

    def take_photo(self):
        """Take high-resolution photo"""
        # Switch to photo mode
        self.set_mode('photo')
        
        # Generate filename with timestamp
        timestamp = time.localtime()
        filename = '/recordings/IMG_{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}.jpg'.format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        
        # Capture and save frame
        frame = self.capture_frame()
        with open(filename, 'wb') as f:
            f.write(frame)
            
        # Switch back to FPV mode
        self.set_mode('fpv')
        
        return filename

    def get_frame(self):
        """Get current frame for streaming"""
        return self.capture_frame()

    def cleanup(self):
        """Cleanup resources"""
        if self.recording:
            self.stop_recording()