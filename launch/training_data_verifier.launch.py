from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_csv_path = DeclareLaunchArgument(
        'csv_path',
        default_value='/home/carson/datasets/self-cap/calibration_tests/sphere_calibrations/link5_3/training_data.csv',
        description='Path to training data CSV file'
    )
    
    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for publishing point cloud'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    declare_publish_all = DeclareLaunchArgument(
        'publish_all_points',
        default_value='true',
        description='Publish all points at once (true) or one by one (false)'
    )
    
    # Create the training data publisher node
    training_data_node = Node(
        package='gentact_ros_tools',
        executable='ee_prediction_verifier',
        name='training_data_publisher',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('csv_path'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'publish_all_points': LaunchConfiguration('publish_all_points'),
        }]
    )

    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )
    
    return LaunchDescription([
        declare_csv_path,
        declare_frame_id,
        declare_publish_rate,
        declare_publish_all,
        training_data_node,
        foxglove_node,
    ])