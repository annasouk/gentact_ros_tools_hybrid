from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='6',
        description='Number of sensors to track'
    )
    
    kp_arg = DeclareLaunchArgument(
        'Kp',
        default_value='0.1',
        description='Proportional gain for PD controller'
    )
    
    kd_arg = DeclareLaunchArgument(
        'Kd',
        default_value='0.01',
        description='Derivative gain for PD controller'
    )
    
    # Create the sensor tracking publisher node
    sensor_tracking_node = Node(
        package='gentact_ros_tools',
        executable='sensor_tracking_pub',
        name='sensor_tracking_publisher',
        parameters=[{
            'num_sensors': LaunchConfiguration('num_sensors'),
            'Kp': LaunchConfiguration('Kp'),
            'Kd': LaunchConfiguration('Kd'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        num_sensors_arg,
        kp_arg,
        kd_arg,
        sensor_tracking_node,
    ]) 