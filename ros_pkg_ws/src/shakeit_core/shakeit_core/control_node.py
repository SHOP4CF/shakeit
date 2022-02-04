import rclpy
import time
from enum import Enum
from rclpy import Future
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose

from shakeit_interfaces.msg import Plot
from anyfeeder_interfaces.srv import StandardInput
from shakeit_interfaces.action import FreeObjects, Pick, SeparateObjects
from shakeit_core.util import create_client_wait_for_service, create_action_client_wait_for_server


class PickStatus(Enum):
    Rejected = 'Rejected'
    Failed = 'Failed'
    Success = 'Success'


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.iterations = self.declare_parameter('iterations', 100)
        self.sim = self.declare_parameter('simulation', False)
        free_objects_action = self.declare_parameter('free_objects_action', 'camera/free_objects')
        pick_object_action = self.declare_parameter('pick_object_action', 'robot/pick')

        self.free_objects_client = create_action_client_wait_for_server(
            self, FreeObjects, free_objects_action.value)
        self.action_client = create_action_client_wait_for_server(
            self, SeparateObjects, 'rl_node/separate_objects')
        self.init_feeder_client = create_client_wait_for_service(
            self, StandardInput, 'feeder/init')
        self.add_objects_client = create_client_wait_for_service(
            self, StandardInput, 'feeder/add')
        self.purge_objects_client = create_client_wait_for_service(
            self, StandardInput, 'feeder/purge')
        self.flip_objects_client = create_client_wait_for_service(
            self, StandardInput, 'feeder/flip')
        self.pick_object_client = create_action_client_wait_for_server(
            self, Pick, pick_object_action.value)

        self.visualizer_pub = self.create_publisher(Plot, 'visualizer', 10)

        self.send_goal_future: Future = None
        self.get_result_future: Future = None

        self.vis = dict(key='pick_time', values=[], labels=['pick_time'])
        self.last_pick_time = time.time()

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def separate_objects(self) -> bool:
        self.send_goal()
        rclpy.spin_until_future_complete(self, self.send_goal_future)
        self.goal_response_callback(self.send_goal_future)
        rclpy.spin_until_future_complete(self, self.get_result_future)
        return self.get_result_callback(self.get_result_future)

    def send_goal(self):
        goal_msg = SeparateObjects.Goal()
        goal_msg.max_actions = 25
        goal_msg.free_objects = 1

        self.get_logger().info(f"Sending goal request...")
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        self.get_logger().debug(f"Received feedback: {feedback.feedback.feedback}")

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Separate object request REJECTED!")
            return

        self.get_logger().info("Separate object request accepted.")
        self.get_result_future = goal_handle.get_result_async()

    def get_result_callback(self, future: Future) -> bool:
        result: SeparateObjects.Result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Objects separated! {result.free_objects} free objects")
            return True
        else:
            status_lookup = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                             4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            res_status = status_lookup[status]
            self.get_logger().warning(f"Failed to separate objects with status: {res_status}.")
            return False

    def get_free_objects(self) -> (bool, int, [Pose]):
        future = self.free_objects_client.send_goal_async(
            FreeObjects.Goal(), feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("free objects request REJECTED!")
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result: FreeObjects.Result = get_result_future.result().result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return result.empty, result.count, result.poses
        else:
            self.get_logger().error("Some error")
            return False, 0, []

    def pick_free_object(self, pose: Pose, retries=5) -> PickStatus:
        request = Pick.Goal(**{'pose': pose})
        future = self.pick_object_client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            if retries > 0:
                self.get_logger().info(f"Pick request rejected. Wait and retry. Retries:{retries}")
                time.sleep(1.5)
                return self.pick_free_object(pose, retries - 1)
            else:
                self.get_logger().warning("Pick object request rejected!")
                return PickStatus.Rejected

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result: Pick.Result = result_future.result().result
        status = result_future.result().status
        self.get_logger().info(f"Status: {status}. Pick message: {result.message}")
        success = status == GoalStatus.STATUS_SUCCEEDED and result.success
        # vis
        if (pick_time := time.time() - self.last_pick_time) < 250:
            self.vis['values'] = [pick_time]
            self.visualizer_pub.publish(Plot(**self.vis))
        self.last_pick_time = time.time()
        return PickStatus.Success if success else PickStatus.Failed

    def init_anyfeeder(self):
        future = self.init_feeder_client.call_async(StandardInput.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Anyfeeder initialized")

    def add_objects(self):
        request = StandardInput.Request()
        request.parameters.repetitions = 5
        request.parameters.speed = 6
        future = self.add_objects_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Objects added")

    def flip_objects(self):
        request = StandardInput.Request()
        request.parameters.repetitions = 1
        request.parameters.speed = 10
        future = self.flip_objects_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Objects flipped")

    def purge_objects(self):
        future = self.purge_objects_client.call_async(StandardInput.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Objects purged")

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.action_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        node.init_anyfeeder()
        iterations = node.iterations.value
        empty, count, _ = node.get_free_objects()
        for i in range(iterations):
            node.get_logger().warning(f"Iteration {i + 1}/{iterations}")
            for _ in range(2):
                if empty or (count < 4 and node.sim):
                    node.add_objects()
            empty, count, points = node.get_free_objects()
            failed_attempts = 0
            while not empty:
                if count > 0:
                    for point in points:
                        pick_status = node.pick_free_object(point)
                        if pick_status == PickStatus.Failed:
                            node.get_logger().warning("Pick failed. Flip and try again.")
                            node.flip_objects()
                            break
                else:
                    if node.separate_objects():
                        failed_attempts = 0
                    else:
                        failed_attempts += 1
                        node.get_logger().warning(f"Failed to separate objects: {failed_attempts} times")
                empty, count, points = node.get_free_objects()

    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
