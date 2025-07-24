from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_urdf_file = PathJoinSubstitution([
        FindPackageShare('gentact_ros_tools'),
        'urdf',
        'fr3_link6.urdf'
    ])
    robot_description = ParameterValue(Command(['cat ', robot_urdf_file]), value_type=str)

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fr3_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='fr3_joint_state_publisher',
        output='screen'
    )

    robot_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    # Sensor     # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for sensor data'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='9600',
        description='Baud rate for serial communication'
    )
    
    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='6',
        description='Number of sensors'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )

    # Create the sensor publisher node
    sensor_publisher_node = Node(
        package='gentact_ros_tools',
        executable='sensor_publisher',
        name='sensor_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'num_sensors': LaunchConfiguration('num_sensors'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
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

    tuning_node = Node(
        package='gentact_ros_tools',
        executable='tuner',
        name='tuner',
        arguments=['0.00007'],
        output='screen'
    )

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='cam_pub',
    )

    camera_node_2 = Node(
        package='camera_tools',
        executable='basic_camera_node',
        name='cam_pub_2',
    )

    pcl_node = Node(
        package='gentact_ros_tools',
        executable='capacitive_pcl',
        name='pcl_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_sensors': LaunchConfiguration('num_sensors'),
            'max_distance': 0.12,
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # serial_port_arg,
        # baud_rate_arg,
        num_sensors_arg,
        # publish_rate_arg,
        kp_arg,
        kd_arg,
        sensor_tracking_node,
        foxglove_bridge,
        robot_st_base_node,
        robot_state_publisher_node,
        pcl_node,
        camera_node_2,
        # rviz_node
    ])
