from multiprocessing import get_logger
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class
from pyfiware import OrionConnector, FiException

from std_msgs.msg import String


class GenericSubscriber(Node):
    url = "http://127.0.0.1:1026"
    itr = 0

    def __init__(self):
        super().__init__('generic_subscriber')
        # ROS Subscribtion
        topic = "topic"
        message_type = get_msg_class(self, topic, include_hidden_topics=True)
        
        self.subscription = self.create_subscription(
            message_type,
            topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Fiware
        self.fiware_manager = OrionConnector(self.url)
        fiware_entity = self.fiware_manager.get(entity_id="urn:ngsi-ld:Store:001", entity_type="Store")
        attributes = {
        "address": {
        "type": "PostalAddress",
        "value": {
            "streetAddress": "Bornholmer Straße 65",
            "addressRegion": "Berlin",
            "addressLocality": "Prenzlauer Berg",
            "postalCode": "10439"
            },
            "metadata": {
                "verified": {
                    "value": True,
                    "type": "Boolean"
                }
            }
        },
        "location": {
            "type": "geo:json",
            "value": {
                "type": "Point",
                "coordinates": [13.3986, 52.5547]
            }
        },
        "name": {
            "type": "Text",
            "value": "Bösebrücke Einkauf"
        }
        }
        if (fiware_entity == None):
            self.fiware_manager.create(element_id="urn:ngsi-ld:Store:001", element_type="Store", **attributes)
        
    def listener_callback(self, msg):
        print(msg)
        self.get_logger().info('I heard: "%s"' % msg.data)
        attributes = {
        "address": {
        "type": "PostalAddress",
        "value": {
            "streetAddress": "Bornholmer Straße 65",
            "addressRegion": "Berlin",
            "addressLocality": "Prenzlauer Berg",
            "postalCode": "10439"
            },
            "metadata": {
                "verified": {
                    "value": True,
                    "type": "Boolean"
                }
            }
        },
        "location": {
            "type": "geo:json",
            "value": {
                "type": "Point",
                "coordinates": [13.3986, self.itr]
            }
        },
        "name": {
            "type": "Text",
            "value": "Bösebrücke Einkauf"
        }
        }
        self.fiware_manager.patch(element_id="urn:ngsi-ld:Store:001", element_type="Store", **attributes)
        self.itr = self.itr + 1

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = GenericSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
