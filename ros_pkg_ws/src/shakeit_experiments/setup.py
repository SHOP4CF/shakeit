from setuptools import setup
from glob import glob
import os

package_name = 'shakeit_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('resource/experiments/*/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bkba',
    maintainer_email='bkba@teknologisk.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'performance_metric = shakeit_experiment.performance_metric:main'
        ],
    },
)
