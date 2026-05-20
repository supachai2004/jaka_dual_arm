import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'jaka_dual_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='supachai',
    maintainer_email='supachai@todo.todo',
    description='Dual arm JAKA A12 cooperative manipulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	   'coordinator = jaka_dual_arm.coordinator:main',
	],
    },
)
