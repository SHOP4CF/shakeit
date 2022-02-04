import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('shakeit_kuka_kr6_workcell'),
                              'urdf',
                              'shakeit_kuka_kr6_workcell.urdf.xacro')
    tmp_urdf_file = os.path.join(get_package_share_directory('shakeit_kuka_kr6_workcell'),
                                 'urdf',
                                 'tmp.urdf')

    opts, input_file_name = xacro.process_args(["-o", tmp_urdf_file, xacro_file])
    xacro._process(input_file_name, vars(opts))

    rviz_cfg = '-d' + os.path.join(get_package_share_directory('shakeit_kuka_kr6_workcell'),
                                   'launch',
                                   'shakeit_kuka_kr6_workcell.rviz')

    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip_argument = DeclareLaunchArgument(
        'robot_ip',
        default_value='10.44.60.115',
        description="ip address (socket) for KUKA robot"
    )
    robot_port = LaunchConfiguration('robot_port')
    robot_port_argument = DeclareLaunchArgument(
        'robot_port',
        default_value='54600',
        description="port (socket) for KUKA robot"
    )
    kuka_adapter_node = Node(
        package='shakeit_adapter',
        executable='kuka_adapter_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'ip': robot_ip,
            'port': robot_port
        }]
    )

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             arguments=[tmp_urdf_file]),
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=[rviz_cfg]),
        # Robot
        robot_ip_argument,
        robot_port_argument,
        kuka_adapter_node,
    ])
