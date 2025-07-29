import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/carson/ros2_ws/src/gentact_ros_tools/install/gentact_ros_tools'
