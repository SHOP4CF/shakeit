import time
import subprocess
from os import path, mkdir

from shutil import copyfile
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def external_py_launch(package_name: str,
                       filename: str,
                       launch_args: dict = None) -> IncludeLaunchDescription:
    pkg_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, '/', filename]),
        launch_arguments=launch_args.items()
    )


def generate_launch_description():
    # configured by user
    shakeit_src = '/ros_pkg_ws/src/'  # <<< must be filled in
    agent_launch = {1: 'dqn_training',
                    2: 'd3qn_training',
                    3: 'a2c_training',
                    4: 'hardcoded_node',
                    5: 'random_node'}[2]
    action_space = get_package_share_directory("shakeit_models") + "/" + {1: '3_actions_real', 2: '30_actions'}[2] + '.json'
    learning_rate = '5e-5'
    epsilon_decay = '0.995'
    batch_size = '64'
    memory_size = '5000'
    activation_function = 'relu'
    iterations = '250'
    sim = True
    headless = False #True  # only relevant if running a sim
    control_node = True

    # generated from user input
    location = path.abspath(__file__)
    launch_name = location.split('/')[-1]
    agent_name = agent_launch.split('_')[0]
    title = '{0}, {1}, {2}'.format(agent_name.upper(),
                                   action_space.split('/')[-1].split('.')[0].replace('_', ' '),
                                   'sim' if sim else 'real')

    if launch_name == 'run_experiment.launch.py':
        print('new_experiment')
        time_signature = time.strftime("%Y%m%d_%H%M%S")
        experiment_name = time_signature + '_' + ('sim' if sim else 'real') + '_' + agent_name
        save_path = shakeit_src + 'shakeit_models/resource/models/' + experiment_name
        mkdir(save_path)
        copyfile(location, save_path + '/' + experiment_name + '.launch.py')
        # get git commit hash
        git_hash = subprocess.check_output(
            ["git", f"--git-dir=/{shakeit_src[:-16]}/.git", "describe", "--always"]).strip()
        with open(save_path + '/commit_hash', 'w') as file:
            file.write(git_hash.decode(encoding='utf-8'))
    else:
        print('restoring_experiment')
        experiment_name = launch_name.split('.')[0]
        save_path = shakeit_src + 'shakeit_models/resource/models/' + experiment_name

    hsv_color_set = """['(0.55, 0.90, 1.00)', '(0.55, 0.90, 1.00)', '(0.55, 0.90, 1.00)', 
                        '(0.60, 0.90, 1.00)', '(0.60, 0.90, 1.00)', '(0.60, 0.90, 1.00)', 
                        '(0.65, 0.90, 1.00)', '(0.65, 0.90, 1.00)', '(0.65, 0.90, 1.00)', 
                        '(0.70, 0.90, 1.00)', '(0.70, 0.90, 1.00)', '(0.70, 0.90, 1.00)', 
                        '(0.75, 0.90, 1.00)', '(0.75, 0.90, 1.00)', '(0.75, 0.90, 1.00)', 
                        '(0.80, 0.90, 1.00)', '(0.80, 0.90, 1.00)', '(0.80, 0.90, 1.00)', 
                        '(0.85, 0.90, 1.00)', '(0.85, 0.90, 1.00)', '(0.85, 0.90, 1.00)', 
                        '(0.90, 0.90, 1.00)', '(0.90, 0.90, 1.00)', '(0.90, 0.90, 1.00)', 
                        '(0.95, 0.90, 1.00)', '(0.95, 0.90, 1.00)', '(0.95, 0.90, 1.00)', 
                        '(0.00, 0.90, 1.00)', '(0.00, 0.90, 1.00)', '(0.00, 0.90, 1.00)']"""

    # external launches
    ext_launch = [external_py_launch(
        package_name='shakeit_models',
        filename=agent_launch + '.launch.py',
        launch_args={
            'save_path': save_path,
            # vis
            'plot_title': title,
            'color_set': hsv_color_set,
            # agent
            'action_space_path': action_space,
            'learning_rate': learning_rate,
            'epsilon_decay': epsilon_decay,
            'batch_size': batch_size,
            'memory_size': memory_size,
            'activation_function': activation_function
            }
        ),
        external_py_launch(
        package_name='shakeit_core',
        filename='sim_pipeline.launch.py' if sim else 'real_pipeline.launch.py',
        launch_args={
            'scene_path': shakeit_src + 'shakeit_sim/assets/vibrating_feeder_real.ttt',
            'save_path': shakeit_src + 'shakeit_models/resource/models/' + experiment_name,
            'action_space_path': action_space,
            'headless': str(headless)
            }
        )]

    if control_node:
        ext_launch.append(external_py_launch(
            package_name='shakeit_core',
            filename='sim_control.launch.py' if sim else 'real_control.launch.py',
            launch_args={
                'iterations': iterations}
        ))

    return LaunchDescription(ext_launch)
