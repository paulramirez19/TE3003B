<<<<<<< Updated upstream
=======
import os
from glob import glob
>>>>>>> Stashed changes
from setuptools import setup

package_name = 'planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
<<<<<<< Updated upstream
        ('share/' + package_name, ['package.xml']),
=======
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy]*'))),
        # Ensure your map files are installed to the share directory
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
>>>>>>> Stashed changes
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Custom A* Path Planner Node for warehouse navigation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< Updated upstream
        	'planner_exe = py_package.planner:main',
=======
            'custom_astar_planner_node = planner.custom_astar_planner_node:main',
            'static_map_publisher_node = planner.static_map_publisher_node:main',
            'test_robot_pose_odom_publisher = test_publishers.test_robot_pose_odom_publisher:main',
>>>>>>> Stashed changes
        ],
    },
)
