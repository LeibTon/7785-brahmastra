from setuptools import find_packages, setup

package_name = 'hammerhead_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
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
            'maze_navigator    = hammerhead_final.maze_navigator:main',
            'wall_follower     = hammerhead_final.wall_follower:main',
            'wall_centering    = hammerhead_final.wall_centering:main',
            'sign_detector     = hammerhead_final.sign_detector:main',
            'turn_controller   = hammerhead_final.turn_controller:main',
        ],
    },
)
