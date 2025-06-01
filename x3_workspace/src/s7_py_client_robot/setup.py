from setuptools import find_packages, setup
import os
from glob import glob

package_name = 's7_py_client_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardo',
    maintainer_email='eduardodavila94@hotmail.com',
    description='Client Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "client_robot1_exe = s7_py_client_robot.client_robot1:main",
            "client_robot2_exe = s7_py_client_robot.client_robot2:main",
            "client_robot3_exe = s7_py_client_robot.client_robot3:main",
            "client_robot4_exe = s7_py_client_robot.client_robot4:main",
        ],
    },
)
