from rclpy.node import Node
from rclpy.client import Client
from rclpy.action import ActionClient

DEFAULT_TIMEOUT = 2.0


def create_client_wait_for_service(node: Node,
                                   srv_type,
                                   srv_name: str) -> Client:
    node.get_logger().info(f"Creating client for free object service on: {srv_name}")
    client = node.create_client(srv_type, srv_name)
    while not client.wait_for_service(timeout_sec=DEFAULT_TIMEOUT):
        node.get_logger().warning(f"{srv_name} service is not available. Waiting again...")
    return client


def create_action_client_wait_for_server(node: Node,
                                         action_type,
                                         action_name: str) -> ActionClient:
    action_client = ActionClient(node, action_type, action_name)
    node.get_logger().info(f"Waiting for action server: {action_name}...")
    while not action_client.wait_for_server(timeout_sec=DEFAULT_TIMEOUT):
        node.get_logger().warning(f"{action_name} actions is not available. Waiting again...")
    return action_client
