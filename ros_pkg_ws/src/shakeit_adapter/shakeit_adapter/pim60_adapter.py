import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shakeit_interfaces.action import FreeObjects
from shakeit_core.util import create_client_wait_for_service
from std_srvs.srv import Trigger
from pim60_interfaces.srv import Common


class PIM60Adapter(Node):

    def __init__(self):
        super().__init__('pim60_adapter')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.trigger_client = create_client_wait_for_service(self, Trigger, 'camera/trigger')
        self.result_client = create_client_wait_for_service(self, Common, 'camera/result')

        self.free_objects_action = ActionServer(
            self, FreeObjects, f'{self.get_name()}/free_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: FreeObjects.Goal):
        self.get_logger().info(f"Received goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Executing goal...")
        result = FreeObjects.Result()

        response: Trigger.Response = self.trigger_client.call(Trigger.Request())
        if response.success:
            response: Common.Response = self.result_client.call(Common.Request())
        if not response.success:
            goal_handle.publish_feedback(FreeObjects.Feedback(**{'feedback': response.message}))
            goal_handle.abort()
            return result

        # TODO(rlh): parse(response-string)
        result.empty = False
        result.count = 1
        result.raw_result = response.message
        goal_handle.succeed()
        return result

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIM60Adapter()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
