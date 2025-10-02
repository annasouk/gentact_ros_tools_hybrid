from logging import Logger
from math import log
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml
import subprocess

def load_config(config_file_name, context):
    package_share = FindPackageShare('gentact_ros_tools').perform(context)
    config_file = os.path.join(package_share, 'config', config_file_name)
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    return config


def build_robot_description(config):
    """Build URDF arguments based on active sensors in config"""
    urdf_args = []
    
    # Loop through all sensors in the config
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('xacro', '') != '':
            xacro_path = sensor_config.get('xacro', '')
            urdf_args.extend([f' {sensor_key}:=', xacro_path])

    # Add end effector mesh if specified in config
    if isinstance(config['robot']['end_effector'], dict) and config['robot']['end_effector'].get('active', False):
        ee_xacro = config['robot']['end_effector'].get('xacro', '')
        if ee_xacro:
            urdf_args.extend([' ee_xacro_file:=', ee_xacro])
    
    urdf_file = PathJoinSubstitution([FindPackageShare('gentact_ros_tools'), config['robot']['robot_xacro']])
    xacro_command = ['xacro ', urdf_file] + urdf_args
    robot_description = ParameterValue(
        Command(xacro_command), 
        value_type=str
    )

    return robot_description

def build_sensor_nodes(config):
    """Build sensor publisher nodes based on active sensors in config"""
    sensor_nodes = []
    
    # Loop through all sensors in the config
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('active', False):

            if sensor_config.get('type', '') == "SPAD":
                # Create a sensor publisher node for each active sensor
                sensor_node = Node(
                    package='gentact_ros_tools',
                    executable='tof_pub_pc',
                    name=f'{sensor_key}_publisher',
                    output='screen',
                    parameters=[{
                        'num_sensors': sensor_config.get('num_sensors', 8),
                        'publish_rate': config['sensors'].get('publish_rate', 30.0),
                    }]
                )
            elif sensor_config.get('type', '') == "SCPS":
                link_name = sensor_key.replace('_skin', '')
                sensor_node = Node(
                    package='gentact_ros_tools',
                    executable='sensor_publisher',
                    name=f'{sensor_key}_publisher',
                    parameters=[{
                        'serial_port': sensor_config.get('port', '/dev/ttyACM0'),
                        'wireless': sensor_config.get('wireless', False),
                        'link_name': link_name,
                        'publish_rate': config['sensors'].get('publish_rate', 30.0),
                        'num_sensors': sensor_config.get('num_sensors', 0),
                        'skin_name': link_name,
                    }],
                    output='screen'
                )
            sensor_nodes.append(sensor_node)

            # udp_node = Node(
            #     package='udp_tof_listener',
            #     executable='udp_grid_listener_array',
            #     name=f'{sensor_key}_udp_listener',
            #     output='screen',
            #     parameters=[{
            #         'num_sensors': sensor_config.get('num_sensors', 8),
            #     }]
            # )
            # sensor_nodes.append(udp_node)
    
    return sensor_nodes

def build_prediction_nodes(config, sensor_key, sensor_config):
    # Prediction nodes based on config for a specific sensor
    prediction_nodes = []
    
    # Add prediction nodes if active
    if 'prediction' in config and config['prediction'].get('active', False):
        prediction_config = config['prediction']
        
        # Add PCL prediction if active
        if prediction_config.get('pcl', {}).get('active', False):
            # Extract link name from sensor key (remove _skin suffix if present)
            link_name = sensor_key.replace('_skin', '')

            if sensor_config.get('alpha', None) is not None:
                alpha = sensor_config.get('alpha', None)
            else:
                alpha = prediction_config['pcl'].get('alpha', 0.003)

            if sensor_config.get('max_distance', None) is not None:
                max_distance = sensor_config.get('max_distance', None)
            else:
                max_distance = prediction_config['pcl'].get('max_distance', 0.1)
            
            # Build parameters dict, only including multiplier if it's defined
            params = {
                'frame_id': link_name,
                'num_sensors': sensor_config.get('num_sensors', 0),
                'skin_name': link_name,
                'alpha': alpha,
                'max_distance': max_distance,
            }
            
            # Only add multiplier parameter if it's actually defined in config
            if 'multiplier' in sensor_config:
                params['multiplier'] = sensor_config['multiplier']
            
            pcl_node = Node(
                package='gentact_ros_tools',
                executable='capacitive_pcl',
                name=f'pcl_prediction_{link_name}',
                parameters=[params],
                output='screen'
            )
            prediction_nodes.append(pcl_node)
        
    

        aggregated_obstacles_node = Node(
            package='gentact_ros_tools',
            executable='closest_obstacle',
            name=f'closest_obstacle_pub',
            output='screen'
        )
        prediction_nodes.append(aggregated_obstacles_node)
    
    return prediction_nodes

'''
def ros1_ros2_bridge():
    
    subprocess.run(["bash;noetic;foxy;ros2 run ros1_bridge dynamic_bridge"], 
                    shell=True) 

def build_avoidance_controller_nodes(config):
    avoidance_controller_nodes = []
    if "avoidance_controller" in config:
        subprocess.run(["bash", 
                        "noetic", 
                        "foxy",
                        "ros2 run ros1_bridge dynamic_bridge"], 
                    shell=False) 
'''
        

def build_joint_relay_nodes(config):
    joint_relay_nodes = []
    if config['robot']['joint_relay']:
        # joint_relay_nodes.append(Node( # Controls the franka to mimic /joint_states_{arm_id}
        #     package='gentact_ros_tools',
        #     executable='franky_relay',
        #     name='franky_relay',
        #     output='screen',
        #     parameters=[{
        #         'robot_ip': config['robot']['robot_ip'],
        #         'arm_id': config['robot']['arm_id'],
        #     }]
        # ))

        joint_relay_nodes.append(Node( # Relays /joint_states to the robot's namespace
            package='gentact_ros_tools',
            executable='panda2fr3',
            name='panda2fr3',
            output='screen',
            parameters=[{
                'arm_id': config['robot']['arm_id'],
            }]
        ))

    return joint_relay_nodes

def build_viz_nodes(config):
    viz_nodes = []
    if config['visualization']['rviz']:
        viz_nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',
            arguments=['-d', config['visualization']['rviz_config']]
        ))


    if config['visualization']['foxglove']:
        viz_nodes.append(Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ))

    return viz_nodes

"""
def build_tuner_nodes(config):
    tuner_nodes = []
    if config['avoidance']['active']:
        tuner_nodes.append(Node(
            package='gentact_ros_tools',
            executable='tuner',
            name='rviz2',
            output='screen',
            parameters=[{
                'config': config['visualization']['rviz_config'],
            }]
        ))
"""
        
def build_cameras_nodes(config):
    cameras_nodes = []
    if config['cameras']['active']:
        cameras_launch_file = config['cameras']['launch_file']
        cameras_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gentact_ros_tools'),
                    'launch',
                    cameras_launch_file
                ])
            ])
        )
        cameras_nodes.append(cameras_launch)
    return cameras_nodes


def launch_setup(context, *args, **kwargs):
    # Get the config file name from launch configuration
    config_file_name = LaunchConfiguration('config').perform(context)
    config = load_config(config_file_name, context)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Build robot description
    robot_description = build_robot_description(config)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{config["robot"]["arm_id"]}_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        remappings=[
            ('/joint_states', '/joint_states_fr3'),
            ('/robot_description', '/robot_description_fr3')
        ]
    )
    robot_st_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base']
    )
    
    
    # Build sensor nodes dynamically from config
    sensor_nodes = build_sensor_nodes(config)
    
    # Build prediction nodes for each active sensor
    prediction_nodes = []
    for sensor_key, sensor_config in config['sensors'].items():
        if isinstance(sensor_config, dict) and sensor_config.get('active', False) and sensor_config.get('type', '') == "SCPS":
            sensor_prediction_nodes = build_prediction_nodes(config, sensor_key, sensor_config)
            prediction_nodes.extend(sensor_prediction_nodes)
            pass
    
    viz_nodes = build_viz_nodes(config)
    camera_nodes = build_cameras_nodes(config)
    joint_relay_nodes = build_joint_relay_nodes(config)

    # Build launch actions list
    launch_actions = [
        robot_state_publisher_node,
        robot_st_base_node,
    ]

    if config['calibration']['xbox']:
        xbox_joint_states = Node(
            package='gentact_ros_tools',
            executable='franky_xbox',
            name='xbox_joint_states',
            output='screen'
        )
        launch_actions.append(xbox_joint_states)

    # Add visualization nodes
    for viz_node in viz_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[viz_node]))

    # Add camera nodes with delays
    for camera_node in camera_nodes:
        launch_actions.append(TimerAction(period=2.0, actions=[camera_node]))

    # Add sensor nodes with delays
    for sensor_node in sensor_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[sensor_node]))
    
    # Add prediction nodes with delays
    for prediction_node in prediction_nodes:
        launch_actions.append(TimerAction(period=1.0, actions=[prediction_node]))

    # Add joint relay nodes with delays
    for joint_relay_node in joint_relay_nodes:
        launch_actions.append(TimerAction(period=5.0, actions=[joint_relay_node]))

    #print("Running ros1_ros2_bridge")
    #ros1_ros2_bridge()

    return launch_actions

def generate_launch_description():

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config',
        default_value='simulation.yaml',
        description='Configuration file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
