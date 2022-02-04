from os import path, mkdir
import time
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
    # configured by the user
    shakeit_src = '/home/bkba/Repos/shakeit/ros_pkg_ws/src/'
    agent_launch = {1: 'dqn_training', 2: 'd3qn_training', 3: 'a2c_training', 4: 'hardcoded_node', 5: 'random_node'}[2]
    action_space = shakeit_src + 'shakeit_models/resource/actions/9_actions.json'

    # generated from user input
    location = path.abspath(__file__)
    launch_name = location.split('/')[-1]

    if launch_name == 'run_experiment.launch.py':
        print('new_experiment')
        time_signature = time.strftime("%Y%m%d_%H%M%S")
        experiment_name = agent_launch.split('_')[0] + '_' + time_signature
        save_path = shakeit_src + 'shakeit_models/resource/models/' + experiment_name
        mkdir(save_path)
        copyfile(location, save_path + '/' + time_signature + '.launch.py')
    else:
        print('restoring_experiment')
        experiment_name = agent_launch.split('_')[0] + '_' + launch_name.split(".")[0]
        save_path = shakeit_src + 'shakeit_models/resource/models/' + experiment_name

    # external launches
    agent_launch = external_py_launch(
        package_name='shakeit_models',
        filename='d3qn_training.launch.py',
        launch_args={
            'save_path': save_path,
            'action_space_path': action_space,
        }
    )

    pipeline_launch = external_py_launch(
        package_name='shakeit_core',
        filename='sim_pipeline.launch.py',
        launch_args={
            'scene_path': shakeit_src + 'shakeit_sim/assets/vibrating_feeder_real.ttt',
            'save_path': shakeit_src + 'shakeit_models/resource/models/' + experiment_name,
            'action_space_path': action_space
        }
    )

    return LaunchDescription([
        agent_launch,
        pipeline_launch
    ])
