import rclpy
import numpy as np
from json import load
from sensor_msgs.msg import Image

from shakeit_models.model import ModelNode
from shakeit_models.agents import A2CAgent


class A2CModelNode(ModelNode):

    def __init__(self):
        super().__init__('A2CModelNode')

        self.image_rows = self.declare_parameter('image_rows', 128)
        self.image_cols = self.declare_parameter('image_cols', 128)
        model_path = self.declare_parameter('model_load', None)
        load_model = self.declare_parameter('load_model', True)
        save_intervals = self.declare_parameter('save_intervals', 20)
        batch_size = self.declare_parameter('batch_size', 10)
        image_format = (self.image_rows.value, self.image_rows.value, 1)
        action_space_path = self.declare_parameter('action_dict_path', '')

        # Actions
        self.action_names = []
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            self.n_actions = len(action_space['actions'])
        self.get_logger().info(f"[init] Model path: {model_path}")

        self.agent = A2CAgent(image_format, self.n_actions,
                              model_path.value, load_model.value, save_intervals.value,
                              batch_size=batch_size.value)

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        image = self.convert_observation(image, (self.image_cols.value, self.image_rows.value))
        # TODO: delete "_process_observation" method from agent and use this instead
        return image


def main(args=None):
    rclpy.init(args=args)
    node = A2CModelNode()

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
