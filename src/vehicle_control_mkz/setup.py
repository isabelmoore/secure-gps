import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vehicle_control_mkz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/*')),
        (os.path.join('share', package_name), glob('data/*')),
        (os.path.join('share', package_name), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_twister = vehicle_control_mkz.nodes.gazebo_twister:main',
            'lat_control = vehicle_control_mkz.nodes.lat_control:main',
            'long_control = vehicle_control_mkz.nodes.long_control:main',
            'odom_pub = vehicle_control_mkz.nodes.odom_pub:main',
            'waypoint_recorder = vehicle_control_mkz.nodes.waypoint_recorder:main',
        ],
    },
)