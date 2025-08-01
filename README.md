# gentact_ros_tools

Install this ros2 package to visualize GenTact sensors as URDFs

# To visualize the sensor on its own:
```bash
ros2 launch gentact_ros_tools self-cap
```

# To calibrate the sensor
```bash
ros2 launch gentact_ros_tools self-cap_calibration.launch.py
```

```bash
noetic \
    foxy \
    ros2 run ros1_bridge dynamic_bridge
```

Then the controller
# 

# Example of compiling a urdf:
```bash
ros2 run xacro xacro urdf/robot/fr3_full_skin.xacro ee_xacro_file:=~/ros2_ws/src/gentact_ros_tools/urdf/end_effectors/sphere_ee.xacro > urdf/compiled/fr3_spheretip.urdf
```

# ML predictions:
```bash
ros2 launch gentact_ros_tools self-cap_predictions.launch.py
```