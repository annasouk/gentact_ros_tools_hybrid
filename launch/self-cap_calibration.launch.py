from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Configure skin files here. '' means no skin.
    # link1_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'skin.xacro'])
    # link2_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'skin.xacro'])
    # link3_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'skin.xacro'])
    # link4_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'skin.xacro'])
    # link5_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'link5_fancy.xacro'])
    # link6_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'skin', 'link6_fancy.xacro'])
    ee_xacro_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'end_effectors', 'large_plate_ee.xacro'])
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'robot', 'fr3_full_skin.xacro'])
    calibration_skin = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'calibration', 'link5.xacro'])
    
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('gentact_ros_tools'), 'config', 'self-cap-calibration.rviz'
    ])  
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, 
            # ' link1_skin:=', link1_skin, 
            # ' link2_skin:=', link2_skin, 
            # ' link3_skin:=', link3_skin, 
            # ' link4_skin:=', link4_skin, 
            # ' link5_skin:=', link5_skin, 
            # ' link6_skin:=', link6_skin,
            ' ee_xacro_file:=', ee_xacro_file]), 
        value_type=str
    )

    skin_description = ParameterValue(Command(['xacro ', calibration_skin]),value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fr3_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    skin_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='skin_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': skin_description}],
        remappings=[
            ('/joint_states', '/joint_states_skin'),
            ('/robot_description', '/robot_description_skin')
        ]
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



    reference_point_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='reference_point_node',
        output='screen',
        arguments=['0.5', '0', '-0.04', '0', '0', '0', 'map', 'reference_point']
    )

    #ros2 run tf2_ros static_transform_publisher 0.47 0 -0.13 1.5707 0 1.78 map calibration_base
    calibration_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='calibration_base_node',
        output='screen',
        arguments=['0.475', '-0.005', '-0.127', '1.5707', '0', '1.8', 'map', 'calibration_base']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )
    
    # Sensor     # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for sensor data'
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
            'num_sensors': LaunchConfiguration('num_sensors'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    camera_node_l = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='cam_pub',
        parameters=[{
            'camera_namespace': "left_cam" 
        }],
    )

    camera_node_r = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='cam_pub',
        parameters=[{
            'camera_namespace': "right_cam" 
        }],
    )

    webcam_node = Node(
        package='camera_tools',
        executable='basic_camera_node',
        name='webcam',
        arguments=['-c', '4']
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    franky_xbox = Node(
        package='gentact_ros_tools',
        executable='franky_xbox',
        name='xbox_joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        serial_port_arg,
        num_sensors_arg,
        publish_rate_arg,
        # foxglove_bridge_node,
        TimerAction(period=1.0, actions=[robot_st_base_node]),
        TimerAction(period=1.0, actions=[reference_point_node]),
        TimerAction(period=1.0, actions=[calibration_base_node]),
        TimerAction(period=1.0, actions=[skin_state_publisher_node]),
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),
        # TimerAction(period=1.0, actions=[joint_state_publisher_node]),
        TimerAction(period=1.0, actions=[rviz_node]),
        TimerAction(period=1.0, actions=[camera_node_r]),
        TimerAction(period=1.0, actions=[camera_node_l]),
        TimerAction(period=1.0, actions=[webcam_node]),
        TimerAction(period=1.0, actions=[sensor_publisher_node]),
        TimerAction(period=1.0, actions=[franky_xbox])
        # TimerAction(period=2.0, actions=[ee_prediction_model_node]),
    ])
