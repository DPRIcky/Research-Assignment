from setuptools import setup
import os
from glob import glob

package_name = 'vehicle_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prajjwal Dutta',
    maintainer_email='your.email@example.com',
    description='Autonomous vehicle navigation with path planning, control, and safety',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_node = vehicle_nav.astar_node:main',
            'rrtstar_node = vehicle_nav.rrtstar_node:main',
            'pid_node = vehicle_nav.pid_node:main',
            'lqr_node = vehicle_nav.lqr_node:main',
            'mpc_node = vehicle_nav.mpc_node:main',
            'cbf_node = vehicle_nav.cbf_node:main',
            'simulator_node = vehicle_nav.simulator_node:main',
            'goal_publisher = vehicle_nav.goal_publisher:main',
            'metrics_node = vehicle_nav.metrics_node:main',
            'turtlesim_bridge = vehicle_nav.turtlesim_bridge:main',
        ],
    },
)
