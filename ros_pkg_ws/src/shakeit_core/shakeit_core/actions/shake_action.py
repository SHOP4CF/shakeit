from rclpy.node import Node

from shakeit_interfaces.srv import Shake
from shakeit_core.actions.action import Action
from shakeit_core.util import create_client_wait_for_service


class ShakeAction(Action):

    def __init__(self, node: Node, shake_service: str, name: str = 'ShakeAction'):
        super().__init__()
        self.name = name
        self.node = node
        self.request = Shake.Request()
        self.shake_service = create_client_wait_for_service(node, Shake, shake_service)
        self.node.get_logger().info(f"[{self.name}] initialized")

    def perform_action(self) -> bool:
        self.node.get_logger().debug(f"[{self.name}] Performing shake action: {self.request}")
        response: Shake.Response = self.shake_service.call(self.request)
        return response.success

    def set_params(self,
                   time: int = None,
                   amplitude: [] = None,
                   frequency: [] = None,
                   phase: [] = None):
        if time:
            self.request.time = time
        if amplitude:
            assert len(amplitude) == 3
            self.request.amplitude = amplitude
        if frequency:
            assert len(frequency) == 3
            self.request.frequency = frequency
        if phase:
            assert len(phase) == 3
            self.request.phase = phase
