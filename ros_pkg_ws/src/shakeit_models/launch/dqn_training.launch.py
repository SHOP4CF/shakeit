from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch nodes for dqn training.

    To run this launch file, call:
      ros2 launch shakeit_models a2c_training.launch.py
    To see the list of arguments and how to pass them:
      ros2 launch shakeit_models a2c_training.launch.py --show-args

    :return:
    """
    model_load = LaunchConfiguration('load_model')
    model_path = LaunchConfiguration('model_load')
    save_interval = LaunchConfiguration('save_interval')
    window_size = LaunchConfiguration('window_size')
    save_plot = LaunchConfiguration('save_plot')
    max_cols = LaunchConfiguration('max_cols')
    figure_dpi = LaunchConfiguration('figure_dpi')
    iterations = LaunchConfiguration('iterations')
    action_dict_path = LaunchConfiguration('action_dict_path')

    model_path_argument = DeclareLaunchArgument(
        'model_load',
        # place your models in ros_pkg_ws/src/shakeit_models/resource/models/model_name
        # use absolute path as an argument
        default_value='',
        description="A path where a model will be loaded/saved from."
    )
    action_dict_path_argument = DeclareLaunchArgument(
        name='action_dict_path',
        default_value='',
        description='A path to an action space stored as a json file.'
    )
    save_plot_argument = DeclareLaunchArgument(
        'save_plot',
        default_value='',
        description="The path to where a plot should be saved every minute. If empty, saving is disabled."
    )
    load_model_argument = DeclareLaunchArgument(
        'load_model',
        default_value='False',
        description="Whether a model should be loaded upon starting the node."
    )
    save_interval_argument = DeclareLaunchArgument(
        'save_interval',
        default_value='10',
        description="An interval (in number of updates) to save a model."
    )
    window_size_argument = DeclareLaunchArgument(
        'window_size',
        default_value='15',
        description="The size of running averaging window in the viz_node."
    )
    max_cols_argument = DeclareLaunchArgument(
        'max_cols',
        default_value='2',
        description="The max number of columns in the dynamic spawn_grid generated in the viz_node."
    )
    figure_dpi_argument = DeclareLaunchArgument(
        'figure_dpi',
        default_value='200',
        description="DPI of the figure plotted in viz_node."
    )
    iterations_argument = DeclareLaunchArgument(
        'iterations',
        default_value='200',
        description="Number of episodes done by the control node."
    )
    dqn_node = Node(
        package='shakeit_models',
        executable='dqn_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'model_path': model_path,
                     'model_load': model_load,
                     'save_intervals': save_interval,
                     'action_space': action_dict_path}]
    )
    viz_node = Node(
        package='shakeit_core',
        executable='viz_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'window_size': window_size,
                     'save_plot': save_plot,
                     'max_cols': max_cols,
                     'figure_dpi': figure_dpi}]
    )
    control_node = Node(
        package='shakeit_core',
        executable='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'iterations': iterations,
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
        model_path_argument,
        load_model_argument,
        save_interval_argument,
        window_size_argument,
        save_plot_argument,
        max_cols_argument,
        figure_dpi_argument,
        iterations_argument,
        action_dict_path_argument,
        dqn_node,
        viz_node,
        control_node
    ])
