from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 确保配置文件被安装到 share 目录
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i6user',
    maintainer_email='i6user@todo.todo',
    description='Lego Vision Service with FoundationPose',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vision_node = my_robot_vision.vision_node:main'
        ],
    },
)