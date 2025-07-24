# Sensor Tracking Publisher

This module implements a PD (Proportional-Derivative) tracker for sensor data that subscribes to `/sensor_raw` and publishes tracked sensor data to `/sensor_tracking`.

## Overview

The `SensorTrackingPublisher` class provides:
- PD control-based tracking of sensor values
- Configurable proportional (Kp) and derivative (Kd) gains
- Smoothing and filtering of raw sensor data
- Difference calculation between consecutive readings

## Features

### PD Controller
The PD controller helps smooth sensor readings and reduce noise by:
- Using proportional gain (Kp) to respond to current error
- Using derivative gain (Kd) to respond to rate of change
- Maintaining state between updates for continuous tracking

### Message Format
The published message on `/sensor_tracking` contains:
- First `num_sensors` values: tracked sensor values
- Next `num_sensors` values: difference values (current - previous)

## Usage

### Running the Node

#### Method 1: Direct execution
```bash
ros2 run gentact_ros_tools sensor_tracking_pub
```

#### Method 2: Using launch file
```bash
ros2 launch gentact_ros_tools sensor_tracking.launch.py
```

#### Method 3: With custom parameters
```bash
ros2 launch gentact_ros_tools sensor_tracking.launch.py Kp:=0.2 Kd:=0.05 num_sensors:=6
```

### Parameters

- `num_sensors` (int, default: 6): Number of sensors to track
- `Kp` (double, default: 0.1): Proportional gain for PD controller
- `Kd` (double, default: 0.01): Derivative gain for PD controller

### Topics

#### Subscribed Topics
- `/sensor_raw` (std_msgs/Int32MultiArray): Raw sensor data

#### Published Topics
- `/sensor_tracking` (std_msgs/Float64MultiArray): Tracked sensor data and differences

## Tuning the PD Controller

### Kp (Proportional Gain)
- Higher values: Faster response, but may cause overshoot
- Lower values: Slower response, more stable
- Typical range: 0.01 - 1.0

### Kd (Derivative Gain)
- Higher values: More damping, reduces overshoot
- Lower values: Less damping, may oscillate
- Typical range: 0.001 - 0.1

### Example Tuning
```bash
# For fast response with some damping
ros2 launch gentact_ros_tools sensor_tracking.launch.py Kp:=0.3 Kd:=0.02

# For slow, stable tracking
ros2 launch gentact_ros_tools sensor_tracking.launch.py Kp:=0.05 Kd:=0.005
```

## Integration with Capacitive PCL

The sensor tracking publisher can be used in conjunction with the capacitive PCL node:

1. Raw sensor data → `/sensor_raw`
2. Sensor tracking publisher processes data → `/sensor_tracking`
3. Capacitive PCL subscribes to tracked data for point cloud generation

This provides smoother, more stable point cloud data for visualization and processing.

## Troubleshooting

### Common Issues

1. **No output on `/sensor_tracking`**: Check if `/sensor_raw` is publishing data
2. **Oscillating values**: Reduce Kp or increase Kd
3. **Slow response**: Increase Kp or reduce Kd
4. **Wrong number of sensors**: Verify `num_sensors` parameter matches actual sensor count

### Debug Information
Enable debug logging to see detailed information:
```bash
ros2 run gentact_ros_tools sensor_tracking_pub --ros-args --log-level debug
``` 