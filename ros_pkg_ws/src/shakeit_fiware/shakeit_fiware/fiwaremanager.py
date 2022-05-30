from multiprocessing import get_logger
from time import sleep
from datetime import datetime
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class
from pyfiware import OrionConnector, FiException

from shakeit_interfaces.msg import Plot
from std_msgs.msg import String


class fiwaremanager(Node):
    url = "http://127.0.0.1:1026"
    entity_type="Asset"
    company = "company-xyz"
    entity_name = "shaky-shaker"
    entity_number = "001"
    entity_id = "urn:ngsi-ld:"+ entity_type + ":" + company + ":" + entity_name + entity_number

    def __init__(self):
        super().__init__('fiwaremanager')
        # ROS Subscribtion
        topic = "visualizer"
        message_type = get_msg_class(self, topic, include_hidden_topics=True)
        message_type = Plot
        self.subscription = self.create_subscription(message_type, topic, self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        # Fiware
        self.fiware_manager = OrionConnector(self.url)
        self.fiware_init()


    def fiware_init(self):    
        fiware_entity = self.fiware_manager.get(entity_id=self.entity_id, entity_type=self.entity_type)
        if (fiware_entity == None):
            timestamp = datetime.utcnow().isoformat()
            attributes = {
            "description": {
                "type": "Property",
                "value": "Smart Self Learning Smart Feeder for Smart Optimizing Smart Shakes"},
            "state": {
                "type": "Property",
                "value": "Available",
                },
            "Rewards": {
                "type": "Property",
                "value": { "reward": 0, "timestamp": timestamp}
                },
            "RewardsMean10": {
                "type": "Property",
                "value": { "reward": 0, "timestamp": timestamp}
                },
            "RewardsMean50": {
                "type": "Property",
                "value": { "reward": 0, "timestamp": timestamp}
                },
            "@context": [
                "https://smartdatamodels.org/context.jsonld"
            ]}
            self.fiware_manager.create(element_id=self.entity_id, element_type=self.entity_type, **attributes)
        
    def listener_callback(self, msg):
        if msg.labels[0] == 'rewards':
            print("Updating entity on Fiware...")
            timestamp = datetime.utcnow().isoformat()
            attributes = {
            "state": {
                "type": "Property",
                "value": "Training",
                    },
            "Rewards": {
                "type": "Property",
                "value": { "reward": msg.values[0], "timestamp": timestamp}
                }}
            self.fiware_manager.patch(element_id=self.entity_id, element_type=self.entity_type, **attributes)
            print("updated Firware")

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = fiwaremanager()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
