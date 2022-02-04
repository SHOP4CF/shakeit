from rclpy.node import Node

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image
from shakeit_interfaces.msg import Update, Plot
from shakeit_interfaces.srv import Act

from shakeit_models.agents.agent import Agent


class ModelNode(Node):

    def __init__(self, name: str):
        super().__init__(name)
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.agent: Agent = None
        self.bridge = CvBridge()

        self.act_service = self.create_service(Act, 'model/act', self.act_callback)
        self.update_sub = self.create_subscription(Update, 'model/update', self.update_callback, 10)
        self.visualize_pub = self.create_publisher(Plot, 'visualizer', 10)
        self.input_image_pub = self.create_publisher(Image, 'model/input_image', 10)

    def act_callback(self, request: Act.Request, response: Act.Response):
        observation = self.prepare_for_nn(request.observation.img)
        action = self.agent.act(observation)
        response.action = action
        self.get_logger().info(f"Sending act response: {response}")
        return response

    def update_callback(self, update: Update):
        to_visualize = self.agent.remember(
            obs=self.prepare_for_nn(update.old_state.img),
            action=update.action,
            reward=update.reward,
            next_obs=self.prepare_for_nn(update.new_state.img),
            done=update.done
        )

        if to_visualize:  # if dictionary is not empty, send to visualizer
            params = dict(key=to_visualize['key'], values=to_visualize['values'], labels=to_visualize['labels'])
            self.visualize_pub.publish(Plot(**params))

    def convert_observation(self, image: Image, size: (int, int)) -> np.ndarray:
        try:  # convert img from ros to cv
            self.get_logger().debug(f"Image encoding: {image.encoding}, h, w: {image.height}, {image.width}")
            if image.encoding == '8UC1':
                image_gray = self.bridge.imgmsg_to_cv2(image)
            else:
                image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
                image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            image_gray = cv2.resize(image_gray, size, interpolation=cv2.INTER_LINEAR)
            image_gray = image_gray[:, :, np.newaxis]

            self.input_image_pub.publish(self.bridge.cv2_to_imgmsg(image_gray))
        except CvBridgeError as e:
            self.get_logger().info(f"[convert_observation] Error: {e}")
            return np.zeros(size)
        return image_gray

    def prepare_for_nn(self, image: Image) -> np.ndarray:
        raise NotImplementedError("Please implement in subclass")

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        self.act_service.destroy()
        self.update_sub.destroy()
        return super().destroy_node()
