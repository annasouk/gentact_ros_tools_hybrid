from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    

    # Declare launch arguments
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    ee_prediction_model_node = Node(
        package='gentact_ros_tools',
        executable='ee_prediction_model_mlp',
        name='ee_prediction_model_mlp',
        output='screen',
        parameters=[{
            'model_path': PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'config', 'mlp_model.pth'])
        }]
    )

    ee_prediction_model_mamba_node = Node(
        package='gentact_ros_tools',
        executable='ee_prediction_model_mamba',
        name='ee_prediction_model_mamba',
        output='screen',
        parameters=[{
            'model_path': PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'config', 'mamba_all.pth']),
            'sensor_frame_id': 'link5_skin'
        }]
    )

    capacitive_pcl_node = Node(
        package='gentact_ros_tools',
        executable='capacitive_pcl',
        name='capacitive_pcl',
        output='screen',
        parameters=[{
            'num_sensors': 6,
            'max_distance': 0.1,
            'sensor_config': PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), 'config', 'link5_sphere_1.json']),
            'frame_id': 'link5',
        }],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # foxglove_bridge_node,
        # ee_prediction_model_node,
        # ee_prediction_model_mamba_node,
        TimerAction(period=1.0, actions=[capacitive_pcl_node]),

    ])
