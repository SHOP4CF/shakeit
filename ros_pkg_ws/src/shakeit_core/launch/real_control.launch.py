from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Start control-node for real setup.

    To run this launch file, call:
      ros2 launch shakeit_core real_control.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_core real_control.launch.py --show-args

    :return:
    """
    iterations = LaunchConfiguration('iterations')
    iterations_argument = DeclareLaunchArgument(
        'iterations',
        default_value='100',
        description="Number of iterations"
    )

    control_node = Node(
        package='shakeit_core',
        executable='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'iterations': iterations,
            'simulation': str(False),
            # Because remapping apparently doesn't work
            'free_objects_action': 'sensopart_adapter/free_objects',
            'pick_object_action': '/kuka_adapter/pick'
        }],
        remappings=[
            ('feeder/init', '/anyfeeder_node/init'),
            ('feeder/add', '/anyfeeder_node/dispense'),
            ('feeder/purge', '/anyfeeder_node/purge'),
            ('feeder/flip', '/anyfeeder_node/flip')
        ]
    )

    return LaunchDescription([
        iterations_argument,
        control_node
    ])
