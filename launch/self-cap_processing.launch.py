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
    ee_xacro_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'end_effectors', 'sphere_ee.xacro'])
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
        arguments=['0.47', '0', '-0.13', '1.5707', '0', '1.78', 'map', 'calibration_base']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    num_sensors_arg = DeclareLaunchArgument(
        'num_sensors',
        default_value='6',
        description='Number of sensors'
    )

    processor_node = Node(
        package='gentact_ros_tools',
        executable='processor',
        name='processor',
        output='screen',
        parameters=[{'num_sensors': LaunchConfiguration('num_sensors')}],
    )   

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        num_sensors_arg,
        foxglove_bridge_node,
        TimerAction(period=1.0, actions=[robot_st_base_node]),
        TimerAction(period=1.0, actions=[reference_point_node]),
        TimerAction(period=1.0, actions=[calibration_base_node]),
        TimerAction(period=1.0, actions=[skin_state_publisher_node]),
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),
        TimerAction(period=1.0, actions=[processor_node]),
    ])
