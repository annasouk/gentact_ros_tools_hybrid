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

## Keyboard TF Controller
Control a `child_frame` in the `/tf` tree with the keyboard.

Run:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run gentact_ros_tools keyboard_tf_broadcaster
```

Parameters (optional):
```bash
ros2 run gentact_ros_tools keyboard_tf_broadcaster --ros-args \
  -p parent_frame:=world \
  -p child_frame:=keyboard_frame \
  -p translation_step:=0.01 \
  -p rotation_step_deg:=5.0
```
Example
`ros2 run gentact_ros_tools keyboard_tf_broadcaster --ros-args -p config:=link6_processing.yaml`

Keys:
- Translation: `w/s` (+x/-x), `a/d` (+y/-y), `r/f` (+z/-z)
- Rotation: `u/o` (+roll/-roll), `i/k` (+pitch/-pitch), `j/l` (+yaw/-yaw)
- Step adjust: `t/g` (trans step up/down), `y/h` (rot step up/down)
- Misc: `space` (reset), `?` (help), `q` (quit)