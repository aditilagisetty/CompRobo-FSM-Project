import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditilagisetty',
    maintainer_email='lagisettyaditi@gmail.com',
    description='Finite state machine project combining Neato robot behaviors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_square = ros_behaviors_fsm.drive_square:main',
            'collision_avoidance = ros_behaviors_fsm.collision_avoidance:main',
            'wall_follower = ros_behaviors_fsm.wall_follower:main',
            'finite_state_controller = ros_behaviors_fsm.finite_state_controller:main',
            'teleop_scan = ros_behaviors_fsm.teleop_scan:main',
            'path_following = ros_behaviors_fsm.path_following:main',
            'icp_localizer = ros_behaviors_fsm.icp_localizer:main',
            'fsm_node = ros_behaviors_fsm.fsm_node:main',
        ],
    },
)
