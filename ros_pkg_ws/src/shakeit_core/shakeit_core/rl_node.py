import rclpy
from json import load
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, CameraInfo
from shakeit_interfaces.msg import Update, Observation, Plot
from shakeit_interfaces.srv import Act
from shakeit_interfaces.action import FreeObjects, SeparateObjects
from shakeit_core.actions import MoveAction
from shakeit_core.util import create_client_wait_for_service, create_action_client_wait_for_server

from time import time
import numpy as np


class RLNode(Node):

    def __init__(self):
        """
        Reinforcement Learning Node.

        Handles requests to separate objects by querying an agent and executing its choices.
        """
        super().__init__('rl_node')
        self.get_logger().info(f"Initializing {self.get_name()}...")
        self.free_objects_action = self.declare_parameter('free_objects_action', 'camera/free_objects').value

        # Load action space
        self.actions = {}
        action_space_path = self.declare_parameter('action_space', [])
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            for key, action in enumerate(action_space['actions']):
                self.actions[key] = MoveAction(self, action['shake_service'], action['name'])
                self.actions[key].set_parameters(**action)

        # Subscribers & Publishers
        self.image_subscriber = self.create_subscription(Image, 'camera/image_color', self.image_callback, 1)
        self.camera_subscriber = self.create_subscription(CameraInfo, 'camera/camera_info', self.camera_callback, 1)
        self.update_pub = self.create_publisher(Update, 'model/update', 10)
        self.vis_pub = self.create_publisher(Plot, 'visualizer', 10)
        self.vis = dict(key='rewards', values=[], labels=['rewards'])

        # ROS actions
        self.act_client = create_client_wait_for_service(self, Act, 'model/act')
        self.free_objects_client = create_action_client_wait_for_server(self, FreeObjects, self.free_objects_action)

        self.separate_objects_action = ActionServer(
            self, SeparateObjects, f'{self.get_name()}/separate_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Observation
        self.latest_image: Image = Image()
        self.camera: CameraInfo = CameraInfo()

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: SeparateObjects.Goal):
        self.get_logger().info(f"Received goal request. free_objects: {goal.free_objects}, "
                               f"max_actions: {goal.max_actions}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        goal: SeparateObjects.Goal = goal_handle.request
        self.get_logger().info(f"Executing goal... free_objects: {goal.free_objects}, max_actions: {goal.max_actions}.")

        free_objects = self.get_free_objects()
        self.send_feedback(goal_handle, free_objects, f"Starting to separate objects.")
        state = self.get_state()
        iteration = 1
        while iteration <= goal.max_actions and not goal_handle.is_cancel_requested:
            start_t = time()
            action_res: Act.Response = self.act_client.call(Act.Request(**{'observation': state}))
            time_taken = time() - start_t
            action = action_res.action
            self.send_feedback(goal_handle, free_objects, f"Performing {self.actions[action]} ({action})")
            new_state, reward, done, free_objects = self.step(action, goal.free_objects, time_taken)
            self.send_update(state, action, reward, new_state, done)
            self.vis['values'] = [reward]
            self.vis_pub.publish(Plot(**self.vis))
            self.send_feedback(goal_handle, free_objects, f"New state, reward: {reward:.6f}, done: {done}")
            if done:
                self.get_logger().warning("DONE!")
                break
            else:
                iteration += 1
                state = new_state

        if goal_handle.is_cancel_requested:
            self.get_logger().info("Goal canceled")
            goal_handle.canceled()
            return SeparateObjects.Result()
        elif iteration > goal.max_actions:
            self.send_feedback(goal_handle, free_objects, f"[ABORT] Max actions performed")
            goal_handle.abort()
            return SeparateObjects.Result()
        else:
            goal_handle.succeed()
            result = SeparateObjects.Result()
            result.free_objects = free_objects
            return result

    def step(self, action: int, free_objects_threshold: int, time_taken: float) \
            -> (Observation, float, bool, int):
        assert action in self.actions, f"Action: {action} not found! " \
                                       f"Possible actions: {self.actions.keys()}"
        if not self.actions[action].perform_action():
            self.get_logger().warning("Failing to complete action for some reason!")
        state = self.get_state()
        free_objects = self.get_free_objects()
        reward = np.clip(a=(free_objects > 0) - time_taken, a_min=-1, a_max=None)
        done = free_objects >= free_objects_threshold

        self.get_logger().info(f"Step taken. State: [some img], Reward: {reward}, "
                               f"Done: {done}, free_objects: {free_objects}")
        return state, reward, done, free_objects

    def image_callback(self, image: Image):
        self.latest_image = image

    def camera_callback(self, info: CameraInfo):
        self.camera = info

    def get_state(self) -> Observation:
        if self.latest_image.width <= 0 or self.latest_image.height <= 0:
            self.get_logger().warning("Latest image doesn't seem to be set!")
        return Observation(**{'img': self.latest_image, 'camera': self.camera})

    def get_free_objects(self) -> int:
        res = self.free_objects_client.send_goal(FreeObjects.Goal())
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"[GET_FREE_OBJECTS] Response: {res}")
            return 0
        free_objects: FreeObjects.Result = res.result
        return free_objects.count

    def send_update(self, old_state: Observation, action: int,
                    reward: float, new_state: Observation, done: bool):
        update = Update()
        update.old_state = old_state
        update.action = action
        update.reward = reward
        update.new_state = new_state
        update.done = done
        self.get_logger().debug(f"Publishing update.")
        self.update_pub.publish(update)

    def send_feedback(self, goal_handle: ServerGoalHandle, free_objects: int, message: str):
        self.get_logger().info(f"Feedback: {message}")
        feedback_msg = SeparateObjects.Feedback()
        feedback_msg.feedback = message
        feedback_msg.free_objects = free_objects
        goal_handle.publish_feedback(feedback_msg)

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.separate_objects_action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RLNode()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
