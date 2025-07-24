import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float64MultiArray
import numpy as np
import time
from typing import List


class SensorTrackingPublisher(Node):
    def __init__(self):
        super().__init__("sensor_tracking_publisher")
        
        # Declare parameters
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('Kp', 0.1)  # Proportional gain
        self.declare_parameter('Kd', 0.01)  # Derivative gain
        self.declare_parameter('baseline_duration', 3.0)  # Baseline collection duration in seconds
        
        # Get parameters
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.baseline_duration = self.get_parameter('baseline_duration').get_parameter_value().double_value
        
        # Set diff_gains to 1.0 for all sensors
        self.diff_gains = np.ones(self.num_sensors)
        
        # Initialize baseline collection
        self.start_time = time.time()
        self.baseline_collected = False
        self.baseline_values = np.zeros(self.num_sensors)
        self.baseline_samples = []
        self.baseline_count = 0
        
        # Initialize PD tracker state
        self.X0 = np.zeros(self.num_sensors)  # Initial state
        self.X = self.X0.copy()  # Current state
        self.last_X = self.X0.copy()  # Previous state
        self.last_err = np.zeros(self.num_sensors)  # Previous error
        self.last_time = time.time()  # Last update time
        
        # Subscribe to raw sensor data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.sensor_callback,
            1
        )
        
        # Create publisher for tracked sensor data
        self.tracking_publisher = self.create_publisher(
            Float64MultiArray,
            '/sensor_tracking',
            10
        )
        
        self.get_logger().info(f"Sensor tracking publisher initialized for {self.num_sensors} sensors")
        self.get_logger().info(f"PD gains: Kp={self.Kp}, Kd={self.Kd}")
        self.get_logger().info(f"Collecting baseline for {self.baseline_duration} seconds...")

    def collect_baseline(self, sensor_values):
        """
        Collect baseline values for the first 3 seconds
        
        Args:
            sensor_values: Current sensor readings
            
        Returns:
            True if baseline collection is complete, False otherwise
        """
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time < self.baseline_duration:
            # Still collecting baseline
            self.baseline_samples.append(sensor_values.copy())
            self.baseline_count += 1
            
            # Log progress every 0.5 seconds
            if self.baseline_count % 10 == 0:  # Assuming ~20Hz update rate
                remaining_time = self.baseline_duration - elapsed_time
                self.get_logger().info(f"Collecting baseline... {remaining_time:.1f}s remaining")
            
            return False
        else:
            # Baseline collection complete
            if not self.baseline_collected:
                self.baseline_values = np.mean(self.baseline_samples, axis=0)
                self.baseline_collected = True
                self.get_logger().info(f"Baseline collection complete! Collected {self.baseline_count} samples")
                self.get_logger().info(f"Baseline values: {self.baseline_values}")
            
            return True

    def update_pd_tracker(self, ref):
        """
        Update PD tracker with new reference values
        
        Args:
            ref: Reference values (numpy array of sensor readings)
            
        Returns:
            Updated state values
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero
        if dt < 1e-6:
            dt = 1e-6
        
        # Calculate error and derivative
        err = ref - self.X
        derr = (err - self.last_err) / dt
        
        # Update state using PD control law
        self.X += self.Kp * err + self.Kd * derr
        
        # Update previous values
        self.last_err = err.copy()
        self.last_time = current_time
        
        return self.X.copy()

    def get_diff(self):
        """
        Calculate the difference between current and previous state
        
        Returns:
            Adjusted difference values
        """
        diff = self.X - self.last_X
        self.last_X = self.X.copy()
        adjusted_diff = np.abs(diff) * self.diff_gains
        return adjusted_diff

    def sensor_callback(self, msg):
        """Callback function for sensor data"""
        try:
            # Get sensor values
            sensor_values = np.array(msg.data, dtype=float)
            
            # Ensure we have the expected number of sensors
            num_available = min(len(sensor_values), self.num_sensors)
            
            # Pad with zeros if we have fewer sensors than expected
            if num_available < self.num_sensors:
                sensor_values = np.pad(sensor_values, (0, self.num_sensors - num_available), 'constant')
            
            # Check if baseline collection is complete
            if not self.collect_baseline(sensor_values):
                # Still collecting baseline, don't publish yet
                return
            
            # Subtract baseline from sensor values
            calibrated_values = sensor_values - self.baseline_values
            
            # Update PD tracker with calibrated values
            tracked_values = self.update_pd_tracker(calibrated_values)
            
            # Calculate differences
            diff_values = self.get_diff()
            
            # Create tracking message with both tracked values and differences
            tracking_msg = Float64MultiArray()
            tracking_msg.data = np.concatenate([tracked_values, diff_values]).tolist()
            
            # Publish tracking data
            self.tracking_publisher.publish(tracking_msg)
            
            self.get_logger().debug(f"Published tracking data for {num_available} sensors")
            self.get_logger().debug(f"Raw values: {sensor_values}")
            self.get_logger().debug(f"Calibrated values: {calibrated_values}")
            self.get_logger().debug(f"Tracked values: {tracked_values}")
            self.get_logger().debug(f"Diff values: {diff_values}")
            
        except Exception as e:
            self.get_logger().error(f"Error in sensor_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    sensor_tracking_pub = SensorTrackingPublisher()
    
    try:
        rclpy.spin(sensor_tracking_pub)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_tracking_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 