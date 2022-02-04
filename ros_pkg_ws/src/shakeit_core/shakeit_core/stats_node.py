import rclpy
import numpy as np
import pickle
from json import load
from rclpy.node import Node
import os.path

from shakeit_interfaces.msg import ActionInfo, Update, Plot


class StatsNode(Node):

    def __init__(self):
        super().__init__('StatsNode')
        self.get_logger().info(f"[{self.get_name()}] Initializing.")
        self.occurrence = self.declare_parameter('occurrence', True)
        self.save_path = self.declare_parameter('save_path', '')
        action_space_path = self.declare_parameter('action_space_path', '')

        # Actions
        self.action_names = []
        with open(action_space_path.value, 'rb') as json_file:
            action_space = load(json_file)
            for key, action in enumerate(action_space['actions']):
                self.action_names.append(action['name'])

        self.action_pubs = {}
        self.stats, loaded = self.load()
        for action in self.action_names:
            self.stats[action] = [] if not loaded else self.stats[action]
            topic = f'{self.get_name()}/stats/{action}'
            self.action_pubs[action] = self.create_publisher(ActionInfo, topic, 10)

        self.update_sub = self.create_subscription(
            Update, 'model/update', self.update_callback, 10)
        self.visualize_pub = self.create_publisher(Plot, 'visualizer', 10)

        self.get_logger().info(f"[{self.get_name()}] Initialized!")

    def update_callback(self, update: Update):
        self.stats[self.action_names[update.action]].append(update.reward)
        for action in self.action_names:
            data = np.array(self.stats[action])
            if data.shape[0] > 0:
                unique, counts = np.unique(data, return_counts=True)
                avg_reward = np.average(data)
                median_reward = np.median(data)
                message = f"{action}: #used: {len(data)}, " \
                          f"avg: {avg_reward:.3f}, median: {median_reward:.3f}"
                if self.occurrence.value:
                    occurrences = ', '.join(f"{u}: {c}" for u, c in zip(unique, counts))
                    message += f", Occurrence: {occurrences}"
                self.get_logger().info(message)
                self.action_pubs[action].publish(ActionInfo(**{'count': len(data),
                                                               'avg': avg_reward,
                                                               'median': median_reward,
                                                               'max': float(np.max(data))}))

        counts = [len(self.stats[action]) for action in self.stats]
        self.visualize_pub.publish(Plot(key='action_counts', values=counts, labels=self.action_names))

    def load(self):
        if os.path.isfile((path := self.save_path.value + '/stats.pickle')):
            self.get_logger().info(f"[{self.get_name()}] File exists loading stats.")
            return pickle.load(open(path, 'rb')), True
        return {}, False

    def save(self):
        if self.save_path.value != '':
            self.get_logger().info(f"[{self.get_name()}] Saving stats to a file.")
            pickle.dump(self.stats, open(self.save_path.value + '/stats.pickle', 'wb'))


def main(args=None):
    rclpy.init(args=args)
    node = StatsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
