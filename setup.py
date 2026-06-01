import os
from glob import glob

from setuptools import setup

package_name = 'vive_libsurvive_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'udev'),
            glob('udev/*')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='fndlrkdl94@gmail.com',
    description='Steam-free HTC VIVE tracking for ROS2 (Galactic) via libsurvive.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survive_tracking_node ='
            ' vive_libsurvive_ros.survive_tracking_node:main',
        ],
    },
)
