import rclpy
import socket
import threading
import math
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shakeit_interfaces.action import Pick
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class KukaAdapter(Node):

    STATES = {-1: 'Unknown',
              0: 'Waiting for new target - ready to move',
              1: 'Received target - on the move',
              2: 'Above target',
              3: 'At target',
              4: 'Out of view',
              5: 'Drop off',
              6: 'Error'}
    END_STATES = [4, 5, 6]
    ERROR_STATES = [6]

    def __init__(self):
        super().__init__('kuka_adapter')
        self.get_logger().info(f"Initializing {self.get_name()}...")

        self.ip = self.declare_parameter('ip', '172.31.1.147')
        self.port = self.declare_parameter('port', 54600)
        self.auto_connect = self.declare_parameter('auto_connect', True)
        self.get_logger().info(
            f"Current parameters:\n"
            f"  address: {self.ip.value}:{self.port.value}\n"
            f"  auto-connect: {self.auto_connect.value}")

        self.program_state = -1
        self.target_id = 1

        self.connected = False
        self.request_socket: socket.socket = None
        self.receive_thread = None

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.wait_for_update_rate = self.create_rate(25)
        self.pick_action = ActionServer(
            self, Pick, f'{self.get_name()}/pick',
            execute_callback=self.execute_pick_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        if self.auto_connect.value:
            self.get_logger().info("Auto connect is on, attempting to connect to robot...")
            self._connect()

        self.get_logger().info(f"{self.get_name()} Initialized! ✅")

    def _connect(self) -> (bool, str):
        if self.connected:
            return False, "Robot is already connected."
        try:
            self.get_logger().info(f"Connecting to robot at: {self.ip.value}:{self.port.value}...")
            self.socket = socket.create_connection((self.ip.value, self.port.value), timeout=5)
            msg = f"Connected to KUKA-robot at {self.ip.value}:{self.port.value}"
            self.get_logger().info(msg)
            self.connected = True
            self.receive_thread = threading.Thread(target=self.read_thread)
            self.receive_thread.start()
            return True, msg
        except socket.error as e:
            msg = f"Failed to connect: {self.ip.value}:{self.port.value}. Error: {e}"
            self.get_logger().error(msg)
            self.connected = False
            self.socket = None
            return False, msg

    def _disconnect(self) -> (bool, str):
        msg = "Connection was already closed!"
        self.connected = False
        if self.socket:
            self.socket.close()
            self.socket = None
            msg = "Connection closed to robot."
        if self.receive_thread is not None:
            self.receive_thread.join()
        self.get_logger().warning(msg)
        return True, msg

    def read_thread(self):
        data = ''
        while self.connected:
            if data.find('</RobotStatus>') == -1:  # data is likely empty; read more data
                try:
                    data += self.socket.recv(1024).decode()
                except socket.error as e:
                    self.get_logger().error(f"Socket error: {e}")
                    # TODO(rlh): perhaps try disconnect/connect?
            else:
                # Ensure that only one message is parsed
                idx = data.find('</RobotStatus') + len('</RobotStatus') + 1
                ready_data = data[:idx]
                data = data[idx:]

                try:
                    xml_root: Element = ET.fromstring(ready_data)
                    for xmlChild in xml_root.iter():
                        if xmlChild.tag == "RobotState":
                            self.parse_robot_state(xmlChild)
                        elif xmlChild.tag == "JointState":
                            self.parse_joint_state(xmlChild)
                        elif xmlChild.tag == "PrgState":
                            self.parse_program_state(xmlChild)
                except ET.ParseError as err:
                    self.get_logger().error(
                        f"ParseError: {err}. Ready_data: {ready_data}. Data: {data}")

    def parse_robot_state(self, xml_node: Element):
        self.get_logger().debug(f"Received XML: {xml_node.attrib}")

    def parse_joint_state(self, xml_node: Element):
        msg = JointState()
        time_now = self.get_clock().now()
        msg.header = Header()
        msg.header.stamp.sec = int(time_now.nanoseconds / 1000000000)
        msg.header.stamp.nanosec = int(time_now.nanoseconds % 1000000000)
        msg.name = ['kr6_r900___joint1', 'kr6_r900___joint2', 'kr6_r900___joint3',
                    'kr6_r900___joint4', 'kr6_r900___joint5', 'kr6_r900___joint6']
        for key in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
            msg.position.append(float(xml_node.attrib[key]) * math.pi / 180.0)

        self.joint_state_publisher.publish(msg)

    def parse_program_state(self, xml_node: Element):
        self.get_logger().debug(f"Received XML: {xml_node.attrib}")
        try:
            return_target_id = int(xml_node.attrib['id'])
            program_state = int(xml_node.attrib['ret_val'])
            if program_state != self.program_state:
                self.program_state = program_state
                self.get_logger().info(
                    f"Current state: {self.STATES[self.program_state]} ({self.program_state}), "
                    f"id: {return_target_id}")
        except ValueError as error:
            self.get_logger().warning(f"Parse error: {error}")

    def goal_callback(self, goal: Pick.Goal):
        self.get_logger().info(f"Received goal request. {goal}. Current state: "
                               f"{self.STATES[self.program_state]} ({self.program_state})")
        if self.program_state == 0:
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Received cancel request for: {goal_handle.goal_id.UUID}.")
        return CancelResponse.ACCEPT

    async def execute_pick_callback(self, goal_handle: ServerGoalHandle):
        goal: Pick.Goal = goal_handle.request
        self.get_logger().info(f"Pick service called. Request: {goal.pose}")

        rot = R.from_quat([goal.pose.orientation.x, goal.pose.orientation.y,
                           goal.pose.orientation.z, goal.pose.orientation.w])
        rot_abc = rot.as_euler('ZYX', degrees=True)

        root = ET.Element('ShakeitRcwCtrl')
        A = rot_abc[0]
        if math.fabs(A) < 90:  # Ensure that the robot doesn't have to rotate more than 90 degrees
            self.get_logger().info(f"A converted from {A} to {A - math.copysign(1, A) * 180}")
            A -= math.copysign(180, A)
        ET.SubElement(root, 'NewTarget', {
            'id': str(self.target_id),
            'X': str(goal.pose.position.x * 1000),
            'Y': str(goal.pose.position.y * 1000),
            'Z': str(goal.pose.position.z * 1000),
            'A': str(A),
            'B': str(rot_abc[1]),
            'C': str(rot_abc[2]),
        })
        robot_request = ET.tostring(root)
        self.get_logger().info(f"Sending: {robot_request}")
        self.socket.send(robot_request)
        self.target_id += 1

        last_program_state = self.program_state
        while self.program_state not in self.END_STATES:
            self.wait_for_update_rate.sleep()
            if last_program_state != self.program_state:
                msg = f"State changed from: {self.STATES[last_program_state]} " \
                      f"to: {self.STATES[self.program_state]}"
                goal_handle.publish_feedback(Pick.Feedback(**{'message': msg}))
                last_program_state = self.program_state

        result = Pick.Result()
        if self.program_state in self.ERROR_STATES:
            goal_handle.abort()
            result.success = False
            result.message = "Some error happen during pick-up!!"
            return result

        result.success = True
        result.message = "The object have now been picked!"
        goal_handle.succeed()
        return result

    def destroy_node(self) -> bool:
        self.get_logger().warning("Closing down")
        self._disconnect()
        self.pick_action.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KukaAdapter()

    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.get_logger().info(f"Ctrl-C detected, shutting {node.get_name()} down!")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
