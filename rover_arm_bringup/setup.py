import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_arm_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pukhraj',
    maintainer_email='pukhraj.pvt1@gmail.com',
    description=(
        'Single-joint ODrive CAN bring-up bridge for a Mars rover 6-DOF '
        'arm. Converts ODrive motor-shaft encoder feedback into calculated '
        'joint-space /joint_states using a fixed gearbox ratio.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_joint_odrive_bridge = '
            'rover_arm_bringup.single_joint_odrive_bridge:main',
        ],
    },
)
