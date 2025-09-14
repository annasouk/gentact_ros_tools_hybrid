# gentact_ros_tools

Install this ros2 package to visualize GenTact sensors as URDFs

# To run the spad skin + sensors:
```bash
ros2 launch gentact_ros_tools spad2.launch.py config:=spad.yaml
```
# To run controller (to get the robot model working):
```bash
ros2 launch gentact_ros_tools franky_xbox
```
# To launch sensors (outside of this package)
```bash 
ros2 run udp_tof_listener udp_grid_listener_array 
```

```bash
ros2 run pointcloud talker
```