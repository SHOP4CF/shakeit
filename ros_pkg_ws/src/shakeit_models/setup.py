import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'shakeit_models'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('resource/models/*/*.launch.py')),
        (os.path.join('share', package_name), glob('resource/actions/*.json'))
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
            'dqn_node = shakeit_models.dqn_node:main',
            'a2c_node = shakeit_models.a2c_node:main',
            'random_node = shakeit_models.random_node:main',
            'hardcoded_node = shakeit_models.hardcoded_node:main',
            'd3qn_node = shakeit_models.d3qn_node:main'
        ],
    },
)
