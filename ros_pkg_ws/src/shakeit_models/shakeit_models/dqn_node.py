import rclpy
import numpy as np
from json import load
from sensor_msgs.msg import Image

from shakeit_models.model import ModelNode
from shakeit_models.agents import DQNAgent


class DQNModelNode(ModelNode):

    def __init__(self):
        super().__init__('DQNModelNode')

        self.image_rows = self.declare_parameter('image_rows', 128)
        self.image_cols = self.declare_parameter('image_cols', 128)
        model_path = self.declare_parameter('model_load')
        load_model = self.declare_parameter('load_model', True)
        save_model = self.declare_parameter('save_model', True)
        save_interval = self.declare_parameter('save_interval', 20)
        action_space_path = self.declare_parameter('action_space', [])

        # Actions
        self.action_names = []
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            self.n_actions = len(action_space['actions'])
            self.get_logger().info('Action space has been successfully defined.')

        self.get_logger().info(f"[init] Model path: {model_path.value}")

        image_format = (self.image_rows.value, self.image_rows.value, 1)
        self.agent = DQNAgent(image_format, self.n_actions,
                              model_path=model_path.value, load_model=load_model.value,
                              save_model=save_model.value, save_interval=save_interval.value)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        image = self.convert_observation(image, (self.image_cols.value, self.image_rows.value))
        return np.expand_dims(image, axis=0) / 255


def main(args=None):
    rclpy.init(args=args)
    node = DQNModelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
