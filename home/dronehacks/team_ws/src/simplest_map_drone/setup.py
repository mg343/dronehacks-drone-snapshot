from glob import glob
from setuptools import setup

package_name = 'simplest_map_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + ['maps/README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='DroneHacks team',
    maintainer_email='noreply@example.com',
    description='Where are we on the course: course position, next checkpoint, nearest wall. Publishes nothing.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['simplest_map_drone = simplest_map_drone.node:main']},
)
