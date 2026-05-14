"""Launch the four robot_control_sim ROS 2 nodes with default parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("robot_control_sim_ros2")
    params = PathJoinSubstitution([pkg, "config", "params.yaml"])
    urdf = PathJoinSubstitution([pkg, "urdf", "robot.urdf.xacro"])
    rviz_cfg = PathJoinSubstitution([pkg, "rviz", "robot_sim.rviz"])

    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", urdf])}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        Node(package="robot_control_sim_ros2", executable="plant_node",
             parameters=[params], output="screen"),
        Node(package="robot_control_sim_ros2", executable="sensor_node",
             parameters=[params], output="screen"),
        Node(package="robot_control_sim_ros2", executable="estimator_node",
             parameters=[params], output="screen"),
        Node(package="robot_control_sim_ros2", executable="controller_node",
             parameters=[params], output="screen"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
