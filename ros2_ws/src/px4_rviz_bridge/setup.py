from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_rviz_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leonard Sfetcu',
    maintainer_email='leonardflorin.sfetcu@nxp.com',
    description='Converts px4_msgs DistanceSensor and SensorOpticalFlow into '
                'RViz-friendly sensor_msgs/Range and visualization_msgs/Marker.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'px4_rviz_bridge = px4_rviz_bridge.px4_rviz_bridge_node:main',
        ],
    },
)
