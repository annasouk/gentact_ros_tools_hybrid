# This node processes the self capacitance data and stores it in a csv file.

import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Int32MultiArray, Float64, Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from typing import List
import tf2_ros
from geometry_msgs.msg import TransformStamped
import csv
import os


class TrainingDataProcessor(Node):
    def __init__(self):
        super().__init__("training_data_processor")

        self.output_file = f'training_data.csv'
        self.declare_parameter('num_sensors', 6)
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.sensor_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.sensor_callback,
            1
        )
        
        # CSV file setup
        self.setup_csv_file()
        
        self.get_logger().info("Processor node initialized with TF2 distance tracking")
    
    def setup_csv_file(self):
        """Initialize the CSV file with headers"""
        # Delete existing file if it exists
        if os.path.exists(self.output_file):
            os.remove(self.output_file)
            self.get_logger().info(f"Deleted existing {self.output_file}")
        
        # Create new file with headers
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Create flat list of headers: timestamp, sensor_0_distance, sensor_1_distance, ..., sensor_0_cc, sensor_1_cc, ...
            headers = ['timestamp']
            headers.extend([f'sensor_{i}_cc' for i in range(self.num_sensors)])
            headers.extend(['ee_x', 'ee_y', 'ee_z', 'closest_sensor'])
            writer.writerow(headers)
            self.get_logger().info(f"Created new {self.output_file} with headers")

    def sensor_callback(self, msg: Int32MultiArray):
        # Create flat list of data: timestamp, sensor_0_distance, sensor_1_distance, ..., sensor_0_cc, sensor_1_cc, ...
        # Use ROS clock time which respects --clock simulation
        ros_time = self.get_clock().now()
        data_log = [ros_time.nanoseconds / 1e9]

        # Add capacitance values for each sensor (flatten msg.data)
        data_log.extend(list(msg.data))

        # Add end effector position
        try:
            ee_pose = self.tf_buffer.lookup_transform(
                'calibration_skin',
                'end_effector_tip',
                rclpy.time.Time()
            )
            data_log.extend([ee_pose.transform.translation.x, ee_pose.transform.translation.y, ee_pose.transform.translation.z])
            data_log.extend([self.closest_sensor])
        except Exception as e:
            self.get_logger().error(f"Error getting end effector pose: {str(e)}")
            return

        self.write_to_csv(data_log)
    
    def write_to_csv(self, log):
        """Write data to CSV file"""
        try:
            with open(self.output_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(log)
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    processor = TrainingDataProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
