import rclpy
import numpy as np
from sensor_msgs.msg import Image
from json import load
from random import randint

from shakeit_models.model import ModelNode


class RandomNode(ModelNode):

    def __init__(self):
        super().__init__('RandomAgent')
        action_space_path = self.declare_parameter('action_space_path', '')
        # Actions
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            self.n_actions = len(action_space['actions'])
            self.get_logger().info('Action space has been successfully defined.')

        self.agent = RandomAgent(self.n_actions)
        self.get_logger().info(f"{self.get_name()} Initialized!")

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        return np.empty((1, 1))


class RandomAgent:

    def __init__(self, n_actions):
        self.n_actions = n_actions

    def act(self, image: np.ndarray) -> int:
        """Return a random action, and fixed repetitions, speed."""
        return randint(0, self.n_actions - 1)

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool):
        pass

    def save(self) -> bool:
        return True

    def load(self) -> bool:
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RandomNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.agent.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
