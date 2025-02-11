from setuptools import setup
import os
from glob import glob
package_name = 'ppc_waypoint_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),  # Include service files GPT told me to add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'mission_node = ppc_waypoint_follower.mission_node:main',
        'behavior_node = ppc_waypoint_follower.behavior_node:main',
        'global_planner = ppc_waypoint_follower.global_planner:main',
        'local_planner = ppc_waypoint_follower.local_planner:main',
        ],
    },
)
