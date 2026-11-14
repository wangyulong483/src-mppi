import os
from glob import glob
from setuptools import setup

package_name = 'origincar_vlm'

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
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    # 注意: 不要加 opencv-python! ROS2 Humble 自带 cv2 (via cv_bridge),
    # pip install opencv-python 会强制升级 numpy→2.x, 导致 cv_bridge 崩溃
    # (_ARRAY_API not found / NumPy 1.x vs 2.x ABI 不兼容)
    zip_safe=True,
    maintainer='wangyulong483',
    maintainer_email='wangyulong483@example.com',
    description='段2 VLM图生文节点：路点触发拍照→云端大模型推理→屏幕显示',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_node = origincar_vlm.vlm_node:main',
        ],
    },
)
