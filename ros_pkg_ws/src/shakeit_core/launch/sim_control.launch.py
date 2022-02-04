from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Start control-node for sim setup.

    To run this launch file, call:
      ros2 launch shakeit_core sim_control.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_core sim_control.launch.py --show-args

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
            'simulation': str(True),
            'free_objects_action': 'free_object_node/free_objects',
            'pick_object_action': '/sim/pick'
        }],
        remappings=[
            ('feeder/init', '/sim/init'),
            ('feeder/add', '/sim/add'),
            ('feeder/purge', '/sim/purge'),
            ('feeder/flip', '/sim/flip')
        ]
    )

    return LaunchDescription([
        iterations_argument,
        control_node
    ])
