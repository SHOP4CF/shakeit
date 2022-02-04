import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'shakeit_core'

setup(
    name=package_name,
    version='0.1.0',
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
            'control_node = shakeit_core.control_node:main',
            'free_object_node = shakeit_core.free_object_node:main',
            'rl_node = shakeit_core.rl_node:main',
            'stats_node = shakeit_core.stats_node:main',
            'viz_node = shakeit_core.viz_node:main'
        ],
    },
)
