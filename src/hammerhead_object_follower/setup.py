from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hammerhead_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'hammerhead_object_follower', 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya_ubuntu',
    maintainer_email='leibton@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'find_object = hammerhead_object_follower.find_object:main',
            'rotate_robot = hammerhead_object_follower.rotate_robot:main'
        ],
    },
)
