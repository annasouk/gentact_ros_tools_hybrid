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
        'fr3_plate_rviz.urdf'
    ])
    robot_description = ParameterValue(Command(['cat ', robot_urdf_file]), value_type=str)

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

    camera_node_2 = Node(
        package='camera_tools',
        executable='basic_camera_node',
        name='cam_pub_2',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # TimerAction(period=0.0, actions=[rviz_node]),
        TimerAction(period=1.0, actions=[robot_state_publisher_node]),
        #TimerAction(period=3.0, actions=[joint_state_publisher_node]),
        TimerAction(period=1.0, actions=[robot_st_base_node]),
        # TimerAction(period=5.0, actions=[tof_listener]),
        TimerAction(period=1.0, actions=[camera_node_2]),
        # TimerAction(period=6.0, actions=[pointcloud_talker]),

    ])
