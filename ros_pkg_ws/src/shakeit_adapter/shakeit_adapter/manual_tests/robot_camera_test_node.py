import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shakeit_interfaces.action import FreeObjects, Trigger
from shakeit_interfaces.action import Pick
from shakeit_core.util import create_action_client_wait_for_server
from action_msgs.msg import GoalStatus


class RobotCameraTestNode(Node):

    def __init__(self):
        super().__init__('robot_camera_test_node')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.pick_action_client = create_action_client_wait_for_server(
            self, Pick, 'kuka_adapter/pick')
        self.free_objects_client = create_action_client_wait_for_server(
            self, FreeObjects, 'sensopart_adapter/free_objects')

        self.test_action = ActionServer(
            self, Trigger, f'{self.get_name()}/test',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: Trigger.Goal):
        self.get_logger().info(f"Received goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Executing goal...")
        result = Trigger.Result()
        result.success = False

        res = self.free_objects_client.send_goal(FreeObjects.Goal())
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            msg = f"[GET_FREE_OBJECTS] Response: {res}"
            self.get_logger().error(msg)
            result.message = msg
            goal_handle.abort()
            return result
        free_objects: FreeObjects.Result = res.result
        if free_objects.count == 0:
            result.message = "No objects to pick!"
            goal_handle.abort()
            return result

        pick_pose = free_objects.poses[0]
        goal_handle.publish_feedback(
            Trigger.Feedback(**{'feedback': f"Picking object: {pick_pose}"}))

        goal = Pick.Goal()
        goal.pose = pick_pose
        res = self.pick_action_client.send_goal(goal, feedback_callback=self.feedback_callback)

        response: Pick.Result = res.result
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        result.success = response.success
        result.message = response.message
        return result

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Received feedback: {feedback.feedback.message}")

    def destroy_node(self) -> bool:
        self.get_logger().warning("Closing down")
        self.test_action.destroy()
        self.free_objects_client.destroy()
        self.pick_action_client.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotCameraTestNode()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
