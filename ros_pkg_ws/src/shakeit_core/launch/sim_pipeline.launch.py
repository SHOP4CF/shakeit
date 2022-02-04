from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Start pipeline for sim setup.

    To run this launch file, call:
      ros2 launch shakeit_core pipeline.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_core pipeline.launch.py --show-args

    :return:
    """
    # paths
    scene_path = LaunchConfiguration('scene_path')
    action_space_path = LaunchConfiguration('action_space_path')
    save_path = LaunchConfiguration('save_path')
    # rest
    extra_stepping = LaunchConfiguration('extra_stepping')
    pick_threshold = LaunchConfiguration('pick_threshold')
    add_new_limit = LaunchConfiguration('add_new_limit')
    print_occurrences = LaunchConfiguration('print_occurrences')
    headless = LaunchConfiguration('headless')

    scene_argument = DeclareLaunchArgument(
        name='scene_path',
        default_value='',
        description="File path to the scene file"
    )
    save_path_argument = DeclareLaunchArgument(
        name='save_path',
        default_value='',
        description='A directory where stats will be saved.'
    )
    action_space_path_argument = DeclareLaunchArgument(
        name='action_space_path',
        default_value='',
        description='A path to an action space stored as a json file.'
    )
    headless_argument = DeclareLaunchArgument(
        name='headless',
        default_value='True',
        description="Run in headless mode or not"
    )
    extra_stepping_argument = DeclareLaunchArgument(
        name='extra_stepping',
        default_value='150',
        description="Extra steps in sim after calling a service"
    )
    pick_threshold_argument = DeclareLaunchArgument(
        name='pick_threshold',
        default_value='0.005',
        description="Max distance between requested pick and objects [meters]"
    )
    add_new_limit_argument = DeclareLaunchArgument(
        name='add_new_limit',
        default_value='5',
        description="random(1, limit) for adding new objects to shaker"
    )
    print_occurrences_argument = DeclareLaunchArgument(
        name='print_occurrences',
        default_value='False',
        description="Turns ON/OFF printout of action occurrences in the stats node."
    )
    rl_node = Node(
        package='shakeit_core',
        executable='rl_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'free_objects_action': 'free_object_node/free_objects',
            'action_space': action_space_path
        }],
        remappings=[
            ('feeder/flip', '/sim/flip'),
            ('feeder/forward', '/sim/forward'),
            ('feeder/backward', '/sim/backward'),
            ('camera/image_color', '/sim/camera/image_color'),
            ('camera/image_info', '/sim/camera/image_info'),
        ]
    )
    stats_node = Node(
        package='shakeit_core',
        executable='stats_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'occurrence': print_occurrences,
                     'action_space_path': action_space_path,
                     'save_path': save_path}]
    )
    free_object_node = Node(
        package='shakeit_core',
        executable='free_object_node',
        output='screen',
        emulate_tty=True,
        parameters=[{}]
    )
    sim_node = Node(
        package='shakeit_sim',
        executable='sim',
        output='screen',
        emulate_tty=True,
        parameters=[{'scene': scene_path,
                     'headless': headless,
                     'extra_stepping': extra_stepping,
                     'pick_threshold': pick_threshold,
                     'add_new_limit': add_new_limit}]
    )

    return LaunchDescription([
        scene_argument,
        save_path_argument,
        action_space_path_argument,
        headless_argument,
        extra_stepping_argument,
        pick_threshold_argument,
        add_new_limit_argument,
        print_occurrences_argument,
        rl_node,
        stats_node,
        free_object_node,
        sim_node
    ])
