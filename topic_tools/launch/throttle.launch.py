import os

import yaml
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def load_throttle_node(context, *args, **kwargs):
    # Resolve the launch configuration to a real folder path
    config_folder = LaunchConfiguration("config_folder").perform(context)
    yaml_file = os.path.join(config_folder, "throttle_topics.yaml")

    with open(yaml_file, "r") as f:
        throttles = yaml.safe_load(f)

    # Extract throttle type and input topics
    throttle_type = throttles.get("throttle_type", "messages")
    input_topics = []
    output_topics = []
    msgs_per_sec_list = []

    for topic in throttles.get("input_topics", []):
        topic_name = topic["topic_name"]
        input_topics.append(topic_name)
        output_topics.append(topic_name + "_telemetry")
        msgs_per_sec_list.append(topic["msgs_per_sec"])

    parameters = [
        {"throttle_type": throttle_type},
        {"input_topics": input_topics},
        {"output_topics": output_topics},
        {"lazy": False},
        {"use_wall_clock": False},
        {"msgs_per_sec": msgs_per_sec_list},  # List of per-topic throttles
    ]

    node = ComposableNode(
        package="topic_tools",
        plugin="topic_tools::ThrottleNode",
        name="throttle_node",
        parameters=parameters,
    )

    return [
        LoadComposableNodes(
            target_container="throttle_container", composable_node_descriptions=[node]
        )
    ]


def generate_launch_description():
    config_folder_arg = DeclareLaunchArgument(
        "config_folder",
        default_value=PathJoinSubstitution(
            [
                EnvironmentVariable("WORKSPACE"),
                "config/system_config/config/",
            ]
        ),
        description="Folder containing throttle YAML files",
    )

    container = ComposableNodeContainer(
        name="throttle_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
    )

    # Use OpaqueFunction to dynamically load YAML and create nodes
    load_node = OpaqueFunction(function=load_throttle_node)

    return LaunchDescription([config_folder_arg, container, load_node])
