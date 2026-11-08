from glob import glob
from setuptools import setup

package_name = 'simplest_fusion_drone'

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
    maintainer='TheSleepyPP',
    maintainer_email='linchenyoung1126@gmail.com',
    description='First sensor fusion: a trust gate from camera, tracking, EKF2, heading and image in front of simplest_square_drone.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['simplest_fusion_drone = simplest_fusion_drone.node:main']},
)
