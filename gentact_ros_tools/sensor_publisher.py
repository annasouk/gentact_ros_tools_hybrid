import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import numpy as np
import time


class SensorPublisher(Node):
    def __init__(self):
        super().__init__("sensor_publisher")
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('num_sensors', 3)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.max_consecutive_errors = 10
        
        # Initialize error tracking
        self.consecutive_errors = 0
        
        # Initialize publisher
        self.publisher = self.create_publisher(
            Int32MultiArray, 
            '/sensor_raw', 
            1
        )
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(self.serial_port, 9600)
            self.get_logger().info(f"Connected to serial port: {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial = None
        
        # Initialize sensor data
        self.last_data = np.zeros(self.num_sensors)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sensor_data)
        
        self.get_logger().info("Sensor publisher initialized")
    
    def read_serial(self):
        """Read data from serial port and parse sensor values"""
        if self.serial is None:
            return self.last_data
            
        try:
            # Check if data is available
            if self.serial.in_waiting > 0:
                data = self.serial.readline()
                # Decode bytes to string and strip whitespace
                data_str = data.decode('utf-8').strip()
                
                # Split by comma and convert to integers
                try:
                    read_numbers = np.array([int(x.strip()) for x in data_str.split(',') if x.strip()])
                    if len(read_numbers) >= self.num_sensors:
                        self.last_data = read_numbers[:self.num_sensors]
                        # Reset error counter on successful read
                        self.consecutive_errors = 0
                except ValueError:
                    # Return last valid data if conversion fails
                    self.consecutive_errors += 1
                    self.get_logger().warn(f"Value conversion error. Consecutive errors: {self.consecutive_errors}")
            else:
                # No data available, return last valid data
                self.consecutive_errors += 1
                self.get_logger().warn(f"No data available. Consecutive errors: {self.consecutive_errors}")
                
        except serial.SerialException as e:
            self.consecutive_errors += 1
            self.get_logger().warn(f"Serial read error: {e}. Consecutive errors: {self.consecutive_errors}")
            time.sleep(0.5)
            
        except Exception as e:
            self.consecutive_errors += 1
            self.get_logger().warn(f"Unexpected error reading serial: {e}. Consecutive errors: {self.consecutive_errors}")
            time.sleep(0.5)

        # Check if we've exceeded the maximum consecutive errors
        if self.consecutive_errors >= self.max_consecutive_errors:
            self.get_logger().error(f"Exceeded maximum consecutive errors ({self.max_consecutive_errors}). Shutting down node.")
            # Schedule node shutdown
            self.create_timer(0.1, self.shutdown_node)
        
        return self.last_data
    
    def shutdown_node(self):
        """Shutdown the node due to excessive errors"""
        self.get_logger().error("Shutting down sensor publisher due to excessive errors")
        rclpy.shutdown()
    
    def publish_sensor_data(self):
        """Read sensor data and publish as ROS2 message"""
        try:
            sensor_values = self.read_serial()
            
            # Create message
            msg = Int32MultiArray()
            msg.data = sensor_values.tolist()
            self.publisher.publish(msg)
            
            # Log for debugging
            self.get_logger().debug(f"Published sensor data: {sensor_values}")
        except Exception as e:
            self.consecutive_errors += 1
            self.get_logger().error(f"Error in publish_sensor_data: {e}. Consecutive errors: {self.consecutive_errors}")
            
            # Check if we've exceeded the maximum consecutive errors
            if self.consecutive_errors >= self.max_consecutive_errors:
                self.get_logger().error(f"Exceeded maximum consecutive errors ({self.max_consecutive_errors}). Shutting down node.")
                # Schedule node shutdown
                self.create_timer(0.1, self.shutdown_node)
    
    def __del__(self):
        """Cleanup serial connection"""
        if hasattr(self, 'serial') and self.serial is not None:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 