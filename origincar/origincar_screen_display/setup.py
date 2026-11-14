import os
from glob import glob

from setuptools import setup


package_name = 'origincar_screen_display'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangyulong483',
    maintainer_email='wangyulong483@example.com',
    description='Independent fullscreen display for QR and VLM results',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'screen_display = origincar_screen_display.screen_display:main',
        ],
    },
)
