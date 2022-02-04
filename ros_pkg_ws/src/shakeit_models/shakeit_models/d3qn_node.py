import rclpy
import numpy as np
from sensor_msgs.msg import Image
from json import load

from shakeit_models.model import ModelNode
from shakeit_models.agents.d3qn_agent import D3QNAgent


class D3QNAgentNode(ModelNode):

    def __init__(self):
        super().__init__('DQNModelNode')

        self.image_rows = self.declare_parameter('image_rows', 128)
        self.image_cols = self.declare_parameter('image_cols', 128)
        action_space_path = self.declare_parameter('action_space', '')
        lr = self.declare_parameter('learning_rate', 1e-4)
        save_path = self.declare_parameter('save_path', '')
        eps_decay = self.declare_parameter('epsilon_decay', 0.99)
        batch_size = self.declare_parameter('batch_size', 32)
        memory_size = self.declare_parameter('memory_size', 5000)
        activation_function = self.declare_parameter('activation_function', 'relu')

        # Actions
        self.action_names = []
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            self.n_actions = len(action_space['actions'])
            self.get_logger().info('Action space has been successfully defined.')

        self.get_logger().info(f"[init] Model path: {save_path.value}")

        image_format = (self.image_rows.value, self.image_rows.value, 1)
        self.agent = D3QNAgent(
            image_format,
            self.n_actions,
            lr=lr.value,
            save_path=save_path.value,
            eps_decay=eps_decay.value,
            batch_size=batch_size.value,
            memory_size=memory_size.value,
            activation=activation_function.value)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        image = self.convert_observation(image, (self.image_cols.value, self.image_rows.value))
        return np.expand_dims(image, axis=0) / 255


def main(args=None):
    rclpy.init(args=args)
    node = D3QNAgentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.agent.save()  # save everything before destroying
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
