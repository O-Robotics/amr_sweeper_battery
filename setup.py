from setuptools import setup
from glob import glob
import os

package_name = 'amr_sweeper_battery'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # install launch files so `ros2 launch` can find them
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Daly BMS battery interface for AMR sweeper over classic CAN (ROS 2).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # must match your launch file
            'amr_sweeper_battery_node = amr_sweeper_battery.node:main',
        ],
    },
)

