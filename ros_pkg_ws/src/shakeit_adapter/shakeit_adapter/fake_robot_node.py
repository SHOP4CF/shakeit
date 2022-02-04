import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shakeit_interfaces.action import Pick


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.waiting_for_pick = False
        self.pick_action = ActionServer(
            self, Pick, f'{self.get_name()}/pick',
            execute_callback=self.execute_pick_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: Pick.Goal):
        self.get_logger().info(f"Received goal request. {goal}.")
        return GoalResponse.REJECT if self.waiting_for_pick else GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_pick_callback(self, goal_handle: ServerGoalHandle):
        goal: Pick.Goal = goal_handle.request
        self.get_logger().info(f"Pick service called. Request: {goal.pose}")

        self.waiting_for_pick = True
        object_picked = False
        while True:
            success = input("Pick the object.\nAre you done [y/n]? ")
            if success.capitalize() == 'Y':
                object_picked = True
                break
            elif success.capitalize() == 'N':
                break
        self.get_logger().info("Move away!")

        result = Pick.Result()
        result.success = object_picked
        if object_picked:
            result.message = "Object manually picked."
            goal_handle.succeed()
        else:
            result.message = "Failed to pick object."
            goal_handle.abort()

        return result

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.pick_action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
