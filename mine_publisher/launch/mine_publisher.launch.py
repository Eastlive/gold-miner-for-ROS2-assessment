# Copyright (c) 2023 Eastlive
# MIT License

# Basic launch file
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

# Add yaml file
import os.path
from ament_index_python.packages import get_package_share_directory

# Add launch config
from launch.substitutions import LaunchConfiguration

# Add launch arguments
from launch.actions import DeclareLaunchArgument

# Add condition
from launch.conditions import UnlessCondition

# Add composable node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    package_path = get_package_share_directory('mine_publisher')
    default_yaml_file_path = os.path.join(
        package_path, 'config', 'mine_publisher.yaml'
    )

    # Declare the launch config
    namespace = LaunchConfiguration('namespace')
    yaml_file_path = LaunchConfiguration('yaml_file_path')
    container_name = LaunchConfiguration('container_name')
    use_external_container = LaunchConfiguration('use_external_container')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    declare_yaml_file_path_cmd = DeclareLaunchArgument(
        'yaml_file_path',
        default_value=default_yaml_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='mine_publisher_container',
        description='The container name'
    )
    declare_use_external_container_cmd = DeclareLaunchArgument(
        'use_external_container',
        default_value='false',
        description='Whether to use an external container'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Create container
    container_node = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        condition=UnlessCondition(use_external_container),
    )

    # Create mine_publisher node
    load_detector = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='mine_publisher',
                plugin='mine_publisher::MinePublisherNode',
                name='mine_publisher',
                namespace=namespace,
                parameters=[yaml_file_path,
                            {'use_sim_time': use_sim_time}]
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_yaml_file_path_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_external_container_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(container_node)
    ld.add_action(load_detector)

    return ld