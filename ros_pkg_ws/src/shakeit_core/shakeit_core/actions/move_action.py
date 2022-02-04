from rclpy.node import Node

from anyfeeder_interfaces.srv import StandardInput
from shakeit_core.actions.action import Action
from shakeit_core.util import create_client_wait_for_service


class MoveAction(Action):

    def __init__(self, node: Node, shake_service: str, name: str = 'MoveAction'):
        super().__init__()
        self.name = name
        self.node = node
        self.request = StandardInput.Request()
        self.set_parameters(**{'repetitions': 1, 'speed': 5})
        self.shake_service = create_client_wait_for_service(node, StandardInput, shake_service)
        self.node.get_logger().info(f"[{self.name}] initialized")

    def perform_action(self) -> bool:
        self.node.get_logger().debug(f"[{self.name}] Performing move action: {self.request}")
        response: StandardInput.Response = self.shake_service.call(self.request)
        return response.status == StandardInput.Response.OK

    def set_parameters(self, **kwargs):
        self.request.parameters.repetitions = kwargs['repetitions']
        self.request.parameters.speed = kwargs['speed']
