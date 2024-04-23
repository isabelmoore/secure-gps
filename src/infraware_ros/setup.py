from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'infraware_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/*')),
        (os.path.join('share', package_name), glob('control_points/*')),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agc4',
    maintainer_email='saminmoosavi.91@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'FilterInfra = infraware_ros.Filter_Infrastructure:main',
        'FilterVehicle = infraware_ros.Filter_Vehicle:main',
        'CPRecorder = infraware_ros.controlPointCollector:main',
        'SensorStacker = infraware_ros.SensorStacker:main',
        'Cartesian = infraware_ros.cartesian_convert:main',
        'GPSSpoof = infraware_ros.spoof_manipulate:main',
        ],
    },
)
