from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def external_py_launch(package_name: str,
                       filename: str,
                       lunch_args: dict = None) -> IncludeLaunchDescription:
    pkg_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, '/', filename]),
        launch_arguments=lunch_args.items()
    )


def generate_launch_description() -> LaunchDescription:
    """
    Start Sensorpart-camera and -adapter node and a special tester-node to bridge to the KUKA-node.

    To run this launch file, call:
      ros2 launch shakeit_adapter test_robot_camera.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_adapter test_robot_camera.launch.py --show-args

    :return:
    """
    sensopart_launch = external_py_launch(
        'sensopart_connector', 'sensopart_connector.launch.py', {'ip': '172.31.1.198'})

    sensopart_adapter_node = Node(
        package='shakeit_adapter',
        executable='sensopart_adapter_node',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('camera/trigger', '/sensopart_connector/extended_trigger'),
        ]
    )

    kuka_adapter_node = Node(
        package='shakeit_adapter',
        executable='kuka_adapter_node',
        output='screen',
        emulate_tty=True,
    )

    test_node = Node(
        package='shakeit_adapter',
        executable='test_robot_camera_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        sensopart_launch,
        sensopart_adapter_node,
        kuka_adapter_node,
        test_node
    ])
