from setuptools import setup

#from ros_pkg_ws.src.shakeit_fiware import shakeit_fiware

package_name = 'shakeit_fiware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miol',
    maintainer_email='miol@teknologisk.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fiware_manager = shakeit_fiware.fiwaremanager:main',
        ],
    },
)
