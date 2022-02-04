import rclpy
import numpy as np
from json import load
from sensor_msgs.msg import Image
from random import choices

from shakeit_models.model import ModelNode


class HardCodedModelNode(ModelNode):

    def __init__(self):
        super().__init__('HardCodedAgent')
        action_space_path = self.declare_parameter('action_space_path', '')
        # Actions
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            self.n_actions = len(action_space['actions'])
            self.get_logger().info('Action space has been successfully defined.')

        self.agent = HardCoded(self.n_actions)
        self.get_logger().info(f"{self.get_name()} Initialized!")

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        return np.empty((1, 1))


class HardCoded:

    def __init__(self, n_actions):
        self.n_actions = n_actions
        self.distribution = [0.35, 0.55, 0.1]  # probs of [flip, forward, backwards], adds to 1

    def act(self, image: np.ndarray) -> int:
        """Return an action based on hard coded probability distribution, and fixed repetitions, speed."""
        return choices([i for i in range(self.n_actions)], self.distribution, k=1)[0]

    def remember(self, obs: np.ndarray, action: int, reward: float, next_obs: np.ndarray, done: bool):
        pass

    def save(self) -> bool:
        return True

    def load(self) -> bool:
        return True


def main(args=None):
    rclpy.init(args=args)
    node = HardCodedModelNode()

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
