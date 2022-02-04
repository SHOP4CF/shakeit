from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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


def generate_launch_description():
    """
    Start SensoPart camera and adapter node.

    To run this launch file, call:
      ros2 launch shakeit_adapter sensopart.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_adapter sensopart.launch.py --show-args

    :return:
    """
    ip = LaunchConfiguration('ip')
    ip_argument = DeclareLaunchArgument(
        'ip',
        default_value='172.31.1.198',
        description="Default ip for the SensoPart Camera"
    )

    sensopart_adapter_node = Node(
        package='shakeit_adapter',
        executable='sensopart_adapter_node',
        output='screen',
        emulate_tty=True,
        parameters=[{}],
        remappings=[
            ('camera/trigger', '/sensopart_connector/extended_trigger'),
        ]
    )
    sensopart_launch = external_py_launch(
        'sensopart_connector', 'sensopart_connector.launch.py', {'ip': ip})

    return LaunchDescription([
        # Camera
        ip_argument,
        sensopart_launch,
        sensopart_adapter_node,
    ])
