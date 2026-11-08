from glob import glob
from setuptools import setup

package_name = 'simplest_course_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='DroneHacks team',
    maintainer_email='noreply@example.com',
    description='Fly the course from the map: the planner writes the step list, fusion and the engine fly it.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['simplest_course_drone = simplest_course_drone.node:main']},
)
