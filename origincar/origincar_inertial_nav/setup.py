import os
from glob import glob
from setuptools import setup

package_name = 'origincar_inertial_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangyulong483',
    maintainer_email='wangyulong483@example.com',
    description='段2 惯导功能包：路径打点、后续回放/状态机',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_recorder = origincar_inertial_nav.path_recorder_node:main',
            'path_follower = origincar_inertial_nav.path_follower_node:main',
            'path_calibrator = origincar_inertial_nav.path_calibrator_node:main',
            'trajectory_pipeline = origincar_inertial_nav.trajectory_pipeline:main',
            'step_tune = origincar_inertial_nav.step_tune:main',
        ],
    },
)
