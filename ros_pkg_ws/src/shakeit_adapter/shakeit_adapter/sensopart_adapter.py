import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shakeit_interfaces.action import FreeObjects
from shakeit_core.util import create_client_wait_for_service
from sensopart_interfaces.srv import ExtendedTrigger
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


class SensopartAdapter(Node):

    CAMERA_SCALE_FACTOR = 0.00001
    CAMERA_ANGLE_FACTOR = 0.01

    def __init__(self):
        super().__init__('sensopart_adapter')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.trigger_client = create_client_wait_for_service(
            self, ExtendedTrigger, 'camera/trigger')

        self.free_objects_action = ActionServer(
            self, FreeObjects, f'{self.get_name()}/free_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: FreeObjects.Goal):
        self.get_logger().info(f"Received free objects request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        result = FreeObjects.Result()

        self.send_feedback(goal_handle, "Trigger camera")
        response: ExtendedTrigger.Response = self.trigger_client.call(ExtendedTrigger.Request())
        if not response.success:
            self.send_feedback(goal_handle, response.message)
            goal_handle.abort()
            return result

        self.send_feedback(goal_handle, f"Trigger success: {response.success}, {response.message}")
        message: str = response.message[1:-1]  # Remove ()
        values = message.split(',')
        result.count = int(values[2])
        for i in range(min(1, result.count)):
            pose = Pose()
            pose.position.x = float(values[3 + i * 3]) * self.CAMERA_SCALE_FACTOR
            pose.position.y = float(values[4 + i * 3]) * self.CAMERA_SCALE_FACTOR
            pose.position.z = float(values[5 + i * 3]) * self.CAMERA_SCALE_FACTOR
            rot: R = R.from_euler('ZYX',
                                  [float(values[8 + i * 3]) * self.CAMERA_ANGLE_FACTOR,
                                   float(values[7 + i * 3]) * self.CAMERA_ANGLE_FACTOR,
                                   float(values[6 + i * 3]) * self.CAMERA_ANGLE_FACTOR],
                                  degrees=True)
            pose.orientation.x = rot.as_quat()[0]
            pose.orientation.y = rot.as_quat()[1]
            pose.orientation.z = rot.as_quat()[2]
            pose.orientation.w = rot.as_quat()[3]

            result.poses.append(pose)

        n_objects = 4
        result.empty = int(values[0]) < n_objects and int(values[1]) < (n_objects * 15)
        result.raw_result = response.message
        goal_handle.succeed()
        return result

    def send_feedback(self, goal_handle: ServerGoalHandle, message: str):
        goal_handle.publish_feedback(FreeObjects.Feedback(**{'feedback': message}))

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.free_objects_action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensopartAdapter()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
