import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    nodes_yaml = LaunchConfiguration("nodes_yaml").perform(context)

    result = []
    if nodes_yaml:
        with open(nodes_yaml, "r") as f:
            specs = yaml.safe_load(f) or []
        for spec in specs:
            remappings = spec.get("remappings")
            if remappings:
                remappings = [tuple(r) for r in remappings]
            result.append(
                Node(
                    package=spec["package"],
                    executable=spec["executable"],
                    namespace=spec.get("namespace"),
                    parameters=spec.get("parameters"),
                    remappings=remappings,
                    arguments=spec.get("arguments"),
                    output=spec.get("output", "screen"),
                )
            )

    result.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    )

    return result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=""),
            DeclareLaunchArgument("nodes_yaml", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
