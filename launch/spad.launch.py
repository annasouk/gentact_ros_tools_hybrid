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
    link1_skin = ''
    link2_skin = ''
    link3_skin = ''
    link4_skin = ''
    link5_skin = '../skin/link5_spad.xacro'
    link6_skin = ''
    ee_mesh_file = ''

    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'urdf', 'robot', 'fr3_full_skin.xacro'])
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, 
            # ' link1_skin:=', link1_skin, 
            # ' link2_skin:=', link2_skin, 
            # ' link3_skin:=', link3_skin, 
            # ' link4_skin:=', link4_skin, 
            ' link5_skin:=', link5_skin,
            #' link6_skin:=', link6_skin,
            #' ee_mesh_file:=', ee_mesh_file
            ]),  
        value_type=str
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

    tof_listener = Node(
        package='udp_tof_listener',
        executable='udp_grid_listener_array',
        name='tof_listener'
    )

    pointcloud_talker = Node(
        package='pointcloud',
        executable='talker',
        name='tof_talker'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # foxglove_bridge,
        TimerAction(period=1.0, actions=[rviz_node]),
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),
        TimerAction(period=1.0, actions=[joint_state_publisher_node]),
        TimerAction(period=1.0, actions=[robot_st_base_node]),
        TimerAction(period=1.0, actions=[tof_listener]),
        TimerAction(period=1.0, actions=[pointcloud_talker]),

    ])
