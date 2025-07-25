import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import socket
import struct
import threading
import time
import numpy as np


class UDPSensorPublisher(Node):
    def __init__(self):
        super().__init__('udp_sensor_publisher')
        
        # Declare parameters
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('buffer_size', 1024)
        self.declare_parameter('timeout_seconds', 5.0)
        
        # Get parameters
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        
        # Initialize publisher
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/sensor_raw', 
            10
        )
        
        # UDP socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.udp_port))
        self.socket.settimeout(1.0)  # 1 second timeout
        
        # Threading
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # Status tracking
        self.last_receive_time = time.time()
        self.receive_count = 0
        
        # Create timer for status updates
        self.status_timer = self.create_timer(5.0, self._status_update)
        
        self.get_logger().info(f"UDP Sensor Publisher started on port {self.udp_port}")
    
    def _receive_loop(self):
        """Main receive loop running in separate thread"""
        while self.running and rclpy.ok():
            try:
                # Receive data
                data, addr = self.socket.recvfrom(self.buffer_size)
                
                # Parse sensor data
                sensor_data = self._parse_sensor_data(data)
                
                if sensor_data is not None:
                    # Publish to ROS2
                    self._publish_sensor_data(sensor_data)
                    
                    # Update status
                    self.last_receive_time = time.time()
                    self.receive_count += 1
                    
                    # Log received data
                    self.get_logger().debug(
                        f"Received from {addr}: device_id={sensor_data['device_id']}, "
                        f"sensors={sensor_data['sensor_values']}"
                    )
                    
            except socket.timeout:
                # Timeout is expected, continue
                continue
            except Exception as e:
                self.get_logger().error(f"Error in receive loop: {e}")
                continue
    
    def _parse_sensor_data(self, data):
        """Parse binary sensor data from ESP32"""
        try:
            # Expected structure: device_id(4) + num_sensors(4) + sensor_values(32)
            if len(data) < 40:  # Minimum size (4+4+32)
                self.get_logger().warn(f"Received data too small: {len(data)} bytes")
                return None
            
            # Unpack binary data
            device_id = struct.unpack('<I', data[0:4])[0]
            num_sensors = struct.unpack('<I', data[4:8])[0]
            
            # Validate number of sensors
            if num_sensors > 8 or num_sensors <= 0:
                self.get_logger().warn(f"Invalid sensor count: {num_sensors}")
                return None
            
            # Parse sensor values (each float is 4 bytes)
            sensor_values = []
            for i in range(num_sensors):
                start_idx = 8 + (i * 4)
                end_idx = start_idx + 4
                if end_idx <= len(data):
                    value = struct.unpack('<f', data[start_idx:end_idx])[0]
                    sensor_values.append(value)
            
            return {
                'device_id': device_id,
                'num_sensors': num_sensors,
                'sensor_values': sensor_values,
                'raw_data': data
            }
            
        except struct.error as e:
            self.get_logger().error(f"Failed to parse sensor data: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing data: {e}")
            return None
    
    def _publish_sensor_data(self, sensor_data):
        """Publish sensor data as ROS2 message"""
        try:
            # Create Float64MultiArray message
            msg = Float64MultiArray()
            msg.data = sensor_data['sensor_values']
            
            # Publish
            self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing sensor data: {e}")
    
    def _status_update(self):
        """Periodic status update"""
        time_since_last = time.time() - self.last_receive_time
        
        if time_since_last > self.timeout_seconds:
            self.get_logger().warn(f"No data received for {time_since_last:.1f} seconds")
        else:
            self.get_logger().info(
                f"Receiving data: {self.receive_count} packets, "
                f"last: {time_since_last:.1f}s ago"
            )
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.running = False
        if hasattr(self, 'socket'):
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    publisher = UDPSensorPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 