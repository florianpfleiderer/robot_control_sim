"""Smoke test: launch the full stack with RViz off and assert nodes come up."""

import os
import time
import unittest

import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("robot_control_sim_ros2")
    launch_file = os.path.join(pkg_share, "launch", "robot_sim.launch.py")

    included = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={"rviz": "false"}.items(),
    )

    return LaunchDescription([
        included,
        launch_testing.actions.ReadyToTest(),
    ])


class TestNodesUp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_observer")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_all_nodes_present(self):
        expected = {"plant_node", "sensor_node", "estimator_node", "controller_node"}
        deadline = time.time() + 10.0
        seen = set()
        while time.time() < deadline:
            names = {n for n, _ in self.node.get_node_names_and_namespaces()}
            seen |= names
            if expected.issubset(seen):
                break
            time.sleep(0.2)
        missing = expected - seen
        self.assertFalse(missing, f"Nodes never appeared: {missing}")
