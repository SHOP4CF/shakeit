import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
from shakeit_interfaces.action import FreeObjects

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class FreeObjectNode(Node):

    def __init__(self):
        super().__init__('free_object_node')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        # ROS
        self.image_subscriber = self.create_subscription(
            Image, 'sim/camera/image_color', self.image_callback, 1)
        self.camera_subscriber = self.create_subscription(
            CameraInfo, 'sim/camera/camera_info', self.camera_callback, 1)
        self.image_publisher = self.create_publisher(
            Image, 'sim/image2', 10)  # this is for debugging purposes solely

        self.free_objects_action = ActionServer(
            self, FreeObjects, f'{self.get_name()}/free_objects',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Internal
        self.latest_image: Image = Image()
        self.camera: CameraInfo = CameraInfo()
        self.bridge = CvBridge()
        self.area_limit = {'min': 680, 'max': 900}
        self.no_pick_x, self.no_pick_y = 95, 45  # pixel ranges where objects cannot be picked
        self.response_cache = None

        self.get_logger().info(f"{self.get_name()} Initialized!")

    def goal_callback(self, goal: FreeObjects.Goal):
        self.get_logger().info(f"Received goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Executing free objects request...")
        result = FreeObjects.Result()

        if self.latest_image == Image():
            self.get_logger().warning(f"Image have not ben set! ABORT")
            result.count = 0
            goal_handle.abort()
            return result

        goal_handle.succeed()
        detected_objects, empty = self.segmentation(self.latest_image)
        result.empty = empty
        result.count = len(detected_objects)
        result.poses = [self.image_to_camera(obj) for obj in detected_objects]
        self.response_cache = result
        return self.response_cache

    def segmentation(self, ros_image) -> ([Point], bool):
        try:  # convert img from ros to cv
            raw_image = self.bridge.imgmsg_to_cv2(ros_image, 'rgb8')
        except CvBridgeError as e:
            self.get_logger().info(e)
            return ros_image
        # color segmentation
        img_hsv = cv2.cvtColor(raw_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img_hsv, np.array([0, 50, 0]), np.array([100, 255, 255]))

        image_gray = cv2.cvtColor(raw_image, cv2.COLOR_RGB2GRAY)
        image_segmented = cv2.bitwise_and(image_gray, image_gray, mask=mask)
        # mask out objects outside the bed
        image_segmented = cv2.rectangle(image_segmented,
                                        (0, 0),
                                        (image_segmented.shape[0], self.no_pick_y),
                                        0, -1)
        image_segmented = cv2.rectangle(image_segmented,
                                        (0, image_segmented.shape[1] - self.no_pick_y),
                                        (image_segmented.shape[0], image_segmented.shape[1]),
                                        0, -1)
        # opening = erosion followed by dilation
        image_eroded = cv2.morphologyEx(image_segmented, cv2.MORPH_ERODE,
                                        kernel=np.ones((5, 5), np.uint8))
        image_opening = cv2.morphologyEx(image_eroded, cv2.MORPH_DILATE,
                                         kernel=np.ones((10, 10), np.uint8))
        # find objects
        contours, _ = cv2.findContours(image_opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info('number of contours {}'.format(len(contours)))
        # filter too-big-blobs out
        filtered_contours = [c for c in contours if
                             self.area_limit['min'] < cv2.contourArea(c) < self.area_limit['max']]
        # draw no-pick line
        cv2.line(image_opening,
                 (self.no_pick_x, 0),
                 (self.no_pick_x, image_opening.shape[1]), 255, thickness=1)
        # is shaker empty
        if len(contours) == 0:  # return, empty shaker
            self.get_logger().info(f'Segmentation done. Shaker is empty.')
            image_contoured = cv2.drawContours(image_opening, contours, -1, 255, 1)
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image_contoured, 'mono8'))
            return [], True
        # find centroids
        detected_objects = []
        for contour in filtered_contours:
            self.get_logger().debug('c_area = {}'.format(cv2.contourArea(contour)))
            moments = cv2.moments(contour)
            cX = moments['m10'] / moments['m00']
            cY = moments['m01'] / moments['m00']
            if cX > self.no_pick_x:
                cv2.circle(image_opening, (int(cX), int(cY)), 2, 255)
                detected_objects.append(Point(**{'x': cX, 'y': cY}))
        # draw contours
        image_contoured = cv2.drawContours(image_opening, filtered_contours, -1, 255, 1)

        self.get_logger().info(f'Segmentation done. Detected {len(detected_objects)} objects.')
        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(image_contoured, 'mono8'))
        return detected_objects, False

    def image_to_camera(self, obj_img: Point) -> Pose:
        Z = 0.233  # camera to feeder distance
        sx = sy = 1.055241729  # meters to pixels conversion ration
        fx = fy = self.camera.k[0]  # focal length
        u0 = v0 = self.camera.k[2]  # principal points
        pose = Pose()
        pose.position.x = -Z*(obj_img.x - u0)/(sx*fx)
        pose.position.y = -Z*(obj_img.y - v0)/(sy*fy)
        return pose

    def image_callback(self, image: Image):
        self.latest_image = image
        self.response_cache = None

    def camera_callback(self, info: CameraInfo):
        self.camera = info

    def destroy_node(self) -> bool:
        self.get_logger().info("Closing down")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FreeObjectNode()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
