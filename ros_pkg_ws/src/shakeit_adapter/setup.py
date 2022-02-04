import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'shakeit_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rlh',
    maintainer_email='rlh@teknologisk.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pim60_adapter_node = shakeit_adapter.pim60_adapter:main',
            'sensopart_adapter_node = shakeit_adapter.sensopart_adapter:main',
            'kuka_adapter_node = shakeit_adapter.kuka_adapter:main',
            'fake_robot_node = shakeit_adapter.fake_robot_node:main',
            'test_robot_camera_node = shakeit_adapter.manual_tests.robot_camera_test_node:main',
        ],
    },
)
