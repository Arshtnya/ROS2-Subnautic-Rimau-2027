from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fake = LaunchConfiguration("fake")
    return LaunchDescription([
        DeclareLaunchArgument("fake", default_value="true",
                              description="jalankan fake_fc (tiruan ESP) untuk uji tanpa robot"),
        Node(package="auv_mission", executable="fake_fc", output="screen",
             condition=IfCondition(fake)),
        Node(package="auv_mission", executable="mission", output="screen"),
    ])