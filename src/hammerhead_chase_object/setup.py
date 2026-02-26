from setuptools import find_packages, setup

package_name = 'hammerhead_chase_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'hammerhead_chase_object', 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aditya',
    maintainer_email='leibton@gatech.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detect_object = hammerhead_chase_object.detect_object:main',
            'get_object_range = hammerhead_chase_object.get_object_range:main',
            'chase_object = hammerhead_chase_object.chase_object:main'
        ],
    },
)
