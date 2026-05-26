from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'jaka_dual_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        # URDF / xacro
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='JAKA dual arm MoveIt2 integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Bridge: /jaka_driver_N/joint_position → /joint_states
            'joint_state_bridge = jaka_dual_arm.joint_state_bridge:main',
            # Action server: FollowJointTrajectory → jaka joint_move service
            'jaka_action_server = jaka_dual_arm.jaka_action_server:main',
        ],
    },
)
