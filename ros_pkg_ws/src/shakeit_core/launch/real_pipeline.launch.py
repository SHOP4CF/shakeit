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
    Start pipeline for real setup.

    To run this launch file, call:
      ros2 launch shakeit_core real_pipeline.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_core real_pipeline.launch.py --show-args

    :return:
    """
    action_space_path = LaunchConfiguration('action_space_path')
    save_path = LaunchConfiguration('save_path')

    action_space_path_argument = DeclareLaunchArgument(
        name='action_space_path',
        default_value='',
        description='A path to an action space stored as a json file.'
    )
    save_path_argument = DeclareLaunchArgument(
        name='save_path',
        default_value='',
        description='A directory where stats will be saved.'
    )

    # AnyFeeder
    # ip = '172.31.1.199'
    device = LaunchConfiguration('device')
    device_argument = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description="Which device to use"
    )
    settling_time = LaunchConfiguration('settling_time')
    settling_time_argument = DeclareLaunchArgument(
        'settling_time',
        default_value='0.5',
        description="The node will wait: settling_delay after each action."
    )
    anyfeeder_node = Node(
        package='anyfeeder_connector',
        executable='anyfeeder_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device': device,
            'settling_time': settling_time
        }]
    )

    # Camera
    ip = LaunchConfiguration('ip')
    ip_argument = DeclareLaunchArgument(
        'ip',
        default_value='172.31.1.198',
        description="Sensopart camera ip address"
    )
    auto_connect = LaunchConfiguration('auto_connect')
    auto_connect_argument = DeclareLaunchArgument(
        'auto_connect',
        default_value='True',
        description="If set to true, the node will automatically try to connect on startup."
    )

    # Pipeline
    rl_node = Node(
        package='shakeit_core',
        executable='rl_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'free_objects_action': 'sensopart_adapter/free_objects',
            'action_space': action_space_path
        }],
        remappings=[
            ('feeder/flip', '/anyfeeder_node/flip'),
            ('feeder/forward', '/anyfeeder_node/feed_forward'),
            ('feeder/backward', '/anyfeeder_node/feed_backward'),
            ('camera/image_color', '/sensopart_connector/image'),
        ]
    )

    print_occurrences = LaunchConfiguration('print_occurrences')
    print_occurrences_argument = DeclareLaunchArgument(
        'print_occurrences',
        default_value='False',
        description="Turns ON/OFF printout of action occurrences in the stats node."
    )
    stats_node = Node(
        package='shakeit_core',
        executable='stats_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'occurrence': print_occurrences,
            'action_space_path': action_space_path,
            'save_path': save_path
        }]
    )

    robot_ip = LaunchConfiguration('robot_ip')
    robot_ip_argument = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.31.1.147',
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
        'sensopart_connector', 'sensopart_connector.launch.py', {
            'ip': ip,
            'auto_connect': auto_connect})

    return LaunchDescription([
        # AnyFeeder
        device_argument,
        settling_time_argument,
        anyfeeder_node,
        # Camera
        ip_argument,
        auto_connect_argument,
        sensopart_launch,
        sensopart_adapter_node,
        # Robot
        robot_ip_argument,
        robot_port_argument,
        kuka_adapter_node,
        # Pipeline
        print_occurrences_argument,
        action_space_path_argument,
        save_path_argument,
        rl_node,
        stats_node,
    ])
