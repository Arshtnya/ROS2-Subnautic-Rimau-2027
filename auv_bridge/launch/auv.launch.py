from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    speed_scale = LaunchConfiguration("speed_scale")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument("speed_scale", default_value="0.5"),
        Node(
            package="auv_bridge",
            executable="bridge",
            name="auv_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "speed_scale": ParameterValue(speed_scale, value_type=float),
            }],
        ),
    ])