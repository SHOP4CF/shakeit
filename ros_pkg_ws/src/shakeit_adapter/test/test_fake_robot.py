# https://answers.ros.org/question/356180/ros2-creating-integration-tests-for-python-nodes/

import os
import sys
import pytest
import unittest

import rclpy
from rclpy.action import ActionClient
import launch
import launch_ros.actions
import launch_testing.actions

from geometry_msgs.msg import Pose
from shakeit_interfaces.action import Pick


@pytest.mark.rostest
def generate_test_description():
    path_to_test = os.path.dirname(__file__)
    path_to_test = path_to_test.replace('test', 'shakeit_adapter')

    fake_robot_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(path_to_test, 'fake_robot_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        remappings=[]
    )

    return (
        launch.LaunchDescription([
            fake_robot_node,
            launch_testing.actions.ReadyToTest()
        ]),
        {
            'fake_robot': fake_robot_node
        }
    )


class TestFakeRobotNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node('test_node')

    def tearDown(self) -> None:
        self.node.destroy_node()

    def create_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose

    def test_fake_robot(self, launch_service, fake_robot, proc_output):
        client = ActionClient(self.node, Pick, 'fake_robot/pick')

        retries, max_attempts = 0, 5
        while not client.wait_for_server(1.0) and retries < max_attempts:
            print("action is not available")
            retries += 1
        self.assertLess(retries, max_attempts, "Pick-action isn't available")

        try:
            goal = Pick.Goal(**{'pose': self.create_pose()})
            future: rclpy.task.Future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            assert future.done(), "Likely timeout while sending goal"
            goal_handle: rclpy.action.client.ClientGoalHandle = future.result()
            assert goal_handle.accepted, f"Request was not accepted: {future}"

            future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            assert not future.done(), "Timeout during pick request"  # TODO: should be changed to "assert future.done()"

        finally:
            client.destroy()
