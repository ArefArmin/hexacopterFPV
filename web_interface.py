"""
Web Interface Module for Hexacopter Control
Author: ArefArmin
Created: 2025-04-08
Last Updated: 2025-04-08 19:53:39 UTC

This module provides a web interface for controlling the hexacopter,
with FPV video streaming, telemetry, and control features.
"""

import network
import socket
import json
import time
import _thread
import machine
import os
import gc
from config import WEB_SERVER, WIFI_CONFIG, CAMERA_SETTINGS
from camera import OV2640

class WebInterface:
    def __init__(self, flight_controller):
        """Initialize web interface and all subsystems"""
        self.flight_controller = flight_controller
        self.camera = OV2640()
        self.clients = {
            'telemetry': set(),
            'video': set(),
            'control': set()
        }
        self.running = True
        
        # Ensure www directory exists
        try:
            os.mkdir('/www')
        except OSError:
            pass
            
        # Setup WiFi Access Point
        self.setup_wifi()
        
        # Initialize servers
        self.servers = self.setup_servers()
        
        # Frame buffer for video streaming
        self.current_frame = None
        self.frame_lock = _thread.allocate_lock()
        
        # Start background tasks
        _thread.start_new_thread(self.video_stream_task, ())
        _thread.start_new_thread(self.telemetry_broadcast_task, ())
        
        print("Web interface initialized")

    def setup_wifi(self):
        """Setup WiFi access point"""
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        
        # Configure access point
        self.ap.config(
            essid=WIFI_CONFIG['SSID'],
            password=WIFI_CONFIG['PASSWORD'],
            channel=WIFI_CONFIG['CHANNEL'],
            authmode=network.AUTH_WPA_WPA2_PSK,
            max_clients=WIFI_CONFIG['MAX_CLIENTS']
        )
        
        while not self.ap.active():
            pass
            
        print("Access Point Configured:")
        print(f"SSID: {WIFI_CONFIG['SSID']}")
        print(f"IP Address: {self.ap.ifconfig()[0]}")

    def setup_servers(self):
        """Setup WebSocket servers for different services"""
        servers = {}
        for service, port in WEB_SERVER['PORTS'].items():
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.bind(('0.0.0.0', port))
            server.listen(5)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            servers[service] = server
            print(f"{service.capitalize()} server started on port {port}")
        return servers

    def send_file(self, client, path):
        """Send file to client with proper HTTP headers"""
        try:
            with open(path, 'rb') as f:
                content = f.read()
                
            # Determine content type
            content_type = 'text/html'
            if path.endswith('.css'):
                content_type = 'text/css'
            elif path.endswith('.js'):
                content_type = 'application/javascript'
            elif path.endswith('.jpg') or path.endswith('.jpeg'):
                content_type = 'image/jpeg'
            elif path.endswith('.png'):
                content_type = 'image/png'
                
            response = (
                f"HTTP/1.1 200 OK\r\n"
                f"Content-Type: {content_type}\r\n"
                f"Content-Length: {len(content)}\r\n"
                "Connection: close\r\n"
                "\r\n"
            ).encode() + content
            
            client.send(response)
            
        except OSError:
            # File not found
            response = (
                "HTTP/1.1 404 Not Found\r\n"
                "Content-Type: text/plain\r\n"
                "Connection: close\r\n"
                "\r\n"
                "404 Not Found"
            ).encode()
            client.send(response)

    def handle_http_request(self, client):
        """Handle HTTP request for static files"""
        try:
            request = client.recv(1024).decode()
            request_line = request.split('\r\n')[0]
            method, path, _ = request_line.split()
            
            if method == 'GET':
                if path == '/' or path == '/index.html':
                    self.send_file(client, '/www/index.html')
                elif path.startswith('/static/'):
                    self.send_file(client, f'/www{path}')
                else:
                    # Default to index for any other path
                    self.send_file(client, '/www/index.html')
                    
        except Exception as e:
            print(f"HTTP request error: {e}")
        finally:
            client.close()

    def handle_websocket_handshake(self, client, service):
        """Handle WebSocket handshake"""
        try:
            request = client.recv(1024).decode('utf-8')
            if "Upgrade: websocket" in request and "Sec-WebSocket-Key" in request:
                for line in request.split('\r\n'):
                    if "Sec-WebSocket-Key" in line:
                        key = line.split(': ')[1]
                        break
                
                import hashlib
                import base64
                GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
                accept_key = base64.b64encode(
                    hashlib.sha1((key + GUID).encode()).digest()
                ).decode()
                
                response = (
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
                )
                client.send(response.encode())
                self.clients[service].add(client)
                return True
                
        except Exception as e:
            print(f"Handshake error: {e}")
        return False

    def encode_websocket_frame(self, data, opcode=0x81):
        """Encode data into WebSocket frame"""
        if isinstance(data, str):
            data = data.encode()
            
        length = len(data)
        frame = bytearray([opcode])
        
        if length < 126:
            frame.append(length)
        elif length < 65536:
            frame.append(126)
            frame.extend(length.to_bytes(2, 'big'))
        else:
            frame.append(127)
            frame.extend(length.to_bytes(8, 'big'))
            
        frame.extend(data)
        return frame

    def decode_websocket_frame(self, data):
        """Decode WebSocket frame"""
        if len(data) < 6:
            return None
            
        payload_start = 2
        payload_length = data[1] & 127
        
        if payload_length == 126:
            payload_start = 4
        elif payload_length == 127:
            payload_start = 10
            
        mask = data[payload_start:payload_start + 4]
        payload_start += 4
        
        payload = bytearray()
        for i in range(payload_start, len(data)):
            payload.append(data[i] ^ mask[(i - payload_start) % 4])
            
        return payload.decode('utf-8')

    def handle_client(self, client, service):
        """Handle client connection for specific service"""
        print(f"New {service} client connected")
        
        # Handle HTTP requests for main server
        if service == 'system':
            self.handle_http_request(client)
            return
            
        # Handle WebSocket connections for other services
        if not self.handle_websocket_handshake(client, service):
            client.close()
            return
            
        try:
            while self.running:
                data = client.recv(1024)
                if not data:
                    break
                    
                decoded = self.decode_websocket_frame(data)
                if decoded:
                    if service == 'control':
                        self.handle_control_message(decoded)
                    elif service == 'video':
                        self.handle_video_message(decoded)
                        
        except Exception as e:
            print(f"{service} client error: {e}")
            
        finally:
            self.clients[service].remove(client)
            client.close()
            print(f"{service} client disconnected")

    def handle_control_message(self, message):
        """Process control messages from clients"""
        try:
            data = json.loads(message)
            
            if 'command' in data:
                if data['command'] == 'arm':
                    self.flight_controller.arm()
                elif data['command'] == 'disarm':
                    self.flight_controller.disarm()
                elif data['command'] == 'emergency':
                    self.flight_controller.emergency_land()
                elif data['command'] == 'setFlightMode':
                    self.flight_controller.set_flight_mode(data['mode'])
                    
            elif all(key in data for key in ['throttle', 'yaw', 'pitch', 'roll']):
                self.flight_controller.process_joystick_input(
                    (data['yaw'], data['throttle']),
                    (data['roll'], data['pitch'])
                )
                
        except json.JSONDecodeError:
            print("Invalid control message format")

    def handle_video_message(self, message):
        """Process video-related commands"""
        try:
            data = json.loads(message)
            
            if data['command'] == 'snapshot':
                filename = self.camera.take_photo()
                self.broadcast_video_event({
                    'type': 'photo_taken',
                    'filename': filename,
                    'timestamp': '2025-04-08 19:53:39',
                    'user': 'ArefArmin'
                })
                
            elif data['command'] == 'startRecording':
                if self.camera.start_recording():
                    self.broadcast_video_event({
                        'type': 'recording_started',
                        'timestamp': '2025-04-08 19:53:39',
                        'user': 'ArefArmin'
                    })
                    
            elif data['command'] == 'stopRecording':
                filename = self.camera.stop_recording()
                if filename:
                    self.broadcast_video_event({
                        'type': 'recording_stopped',
                        'filename': filename,
                        'timestamp': '2025-04-08 19:53:39',
                        'user': 'ArefArmin'
                    })
                    
        except json.JSONDecodeError:
            print("Invalid video command format")

    def video_stream_task(self):
        """Background task for video streaming"""
        while self.running:
            try:
                # Capture new frame
                with self.frame_lock:
                    self.current_frame = self.camera.get_frame()
                
                # Stream to all video clients
                if self.current_frame:
                    frame_data = self.encode_websocket_frame(
                        self.current_frame,
                        opcode=0x82  # Binary frame
                    )
                    self.broadcast_to_clients('video', frame_data)
                
                # Maintain FPV frame rate
                time.sleep_ms(int(1000 / CAMERA_SETTINGS['fpv']['fps']))
                
            except Exception as e:
                print(f"Video stream error: {e}")
                time.sleep_ms(100)

    def telemetry_broadcast_task(self):
        """Background task for broadcasting telemetry"""
        while self.running:
            try:
                telemetry = self.flight_controller.get_telemetry()
                frame = self.encode_websocket_frame(telemetry)
                self.broadcast_to_clients('telemetry', frame)
                time.sleep_ms(50)  # 20Hz update rate
                
            except Exception as e:
                print(f"Telemetry broadcast error: {e}")
                time.sleep_ms(100)

    def broadcast_to_clients(self, service, data):
        """Broadcast data to all clients of a service"""
        dead_clients = set()
        
        for client in self.clients[service]:
            try:
                client.send(data)
            except:
                dead_clients.add(client)
                
        # Remove dead clients
        self.clients[service] -= dead_clients

    def broadcast_video_event(self, event_data):
        """Broadcast video-related events to clients"""
        frame = self.encode_websocket_frame(json.dumps(event_data))
        self.broadcast_to_clients('video', frame)

    def start(self):
        """Start the web interface servers"""
        print("Starting web interface servers...")
        
        # Start service handlers in separate threads
        for service, server in self.servers.items():
            _thread.start_new_thread(self.service_handler, (service, server))
            
        # Main thread keeps running for background tasks
        try:
            while self.running:
                gc.collect()  # Memory management
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def service_handler(self, service, server):
        """Handle connections for a specific service"""
        while self.running:
            try:
                client, addr = server.accept()
                _thread.start_new_thread(
                    self.handle_client,
                    (client, service)
                )
            except Exception as e:
                print(f"{service} server error: {e}")

    def stop(self):
        """Stop all servers and cleanup"""
        print("Stopping web interface...")
        self.running = False
        
        # Close all client connections
        for clients in self.clients.values():
            for client in clients:
                try:
                    client.close()
                except:
                    pass
                    
        # Close all servers
        for server in self.servers.values():
            try:
                server.close()
            except:
                pass
                
        # Cleanup camera
        self.camera.cleanup()