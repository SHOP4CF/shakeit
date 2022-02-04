from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch nodes for d3qn training.

    To run this launch file, call: ros2 launch shakeit_models d3q_training.launch.py
    :return:
    """
    # shared
    save_path = LaunchConfiguration('save_path')
    # agent
    action_space_path = LaunchConfiguration('action_space_path')
    learning_rate = LaunchConfiguration('learning_rate')
    epsilon_decay = LaunchConfiguration('epsilon_decay')
    batch_size = LaunchConfiguration('batch_size')
    memory_size = LaunchConfiguration('memory_size')
    activation_function = LaunchConfiguration('activation_function')
    # viz node
    plot_title = LaunchConfiguration('plot_title')
    window_size = LaunchConfiguration('window_size')
    max_cols = LaunchConfiguration('max_cols')
    figure_dpi = LaunchConfiguration('figure_dpi')
    color_set = LaunchConfiguration('color_set')

    # shared
    save_path_argument = DeclareLaunchArgument(
        'save_path',
        default_value='',
        description='Absolute path to the model.'
    )

    # agent
    action_space_path_argument = DeclareLaunchArgument(
        name='action_space_path',
        default_value='',
        description='A path to an action space stored as a json file.'
    )
    learning_rate_argument = DeclareLaunchArgument(
        name='learning_rate',
        default_value='1e-4',
        description='The learning rate used by the optimizer.'
    )
    epsilon_decay_argument = DeclareLaunchArgument(
        name='epsilon_decay',
        default_value='0.99',
        description='The ration of epsilon decay, exploration-exploitation tradeoff.'
    )
    batch_size_argument = DeclareLaunchArgument(
        name='batch_size',
        default_value='32',
        description='The number of samples used for updating the weights.'
    )
    memory_size_argument = DeclareLaunchArgument(
        name='memory_size',
        default_value='5000',
        description='The max number of memories in the replay buffer.'
    )
    activation_function_argument = DeclareLaunchArgument(
        name='activation_function',
        default_value='relu',
        description='The activation function used across the model layers.'
    )

    # viz node
    plot_title_argument = DeclareLaunchArgument(
        'plot_title',
        default_value='Training progress',
        description='Title for the training plot.'
    )
    window_size_argument = DeclareLaunchArgument(
        'window_size',
        default_value='[10, 50, 100]',
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
    color_set_argument = DeclareLaunchArgument(
        'color_set',
        default_value='[]',
        description="The color palette for plots in rgb scale. E.g. [(1., 0., 0.), (0., 1., 0.),(0., 0., 1.)]"
    )

    # nodes
    d3qn_node = Node(
        package='shakeit_models',
        executable='d3qn_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'save_path': save_path,
            'action_space': action_space_path,
            'learning_rate': learning_rate,
            'epsilon_decay': epsilon_decay,
            'batch_size': batch_size,
            'memory_size': memory_size,
            'activation_function': activation_function}]
    )
    viz_node = Node(
        package='shakeit_core',
        executable='viz_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'save_plot': save_path,
            'title': plot_title,
            'window_size': window_size,
            'max_cols': max_cols,
            'figure_dpi': figure_dpi,
            'color_set': color_set}]
    )

    return LaunchDescription([
        # shared
        save_path_argument,
        # agent
        action_space_path_argument,
        learning_rate_argument,
        epsilon_decay_argument,
        batch_size_argument,
        memory_size_argument,
        activation_function_argument,
        # viz
        plot_title_argument,
        window_size_argument,
        max_cols_argument,
        figure_dpi_argument,
        color_set_argument,
        # nodes
        d3qn_node,
        viz_node
    ])
