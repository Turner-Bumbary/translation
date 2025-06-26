from setuptools import find_packages, setup

package_name = 'translation'

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
    maintainer='Turner Bumbary',
    maintainer_email='tbumbary3@gatech.edu',
    description='translates ROS2 messages into uORB topics for PX4 Pixhawk 6X controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'translator = translation.translator:main',
            'filtered_velocity = translation.filtered_velocities:main',
            'position_only = translation.position_only:main',
        ],
    },
)
