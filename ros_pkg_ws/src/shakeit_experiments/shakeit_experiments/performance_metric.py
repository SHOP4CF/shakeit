import rclpy
from rclpy.node import Node
from shakeit_interfaces.msg import Plot, Update


class StatsNode(Node):

    def __init__(self):
        super().__init__('StatsNode')
        self.update_sub = self.create_subscription(Update, 'model/update', self.update_callback, 10)
        self.visualize_pub = self.create_publisher(Plot, 'visualizer', 10)
        self.actions = 0

    def update_callback(self, update: Update):
        self.actions += 1
