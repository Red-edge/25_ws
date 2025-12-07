from setuptools import setup
import os
from glob import glob

package_name = 'tb4_autonav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 注册为 ROS2 包（ament_index）
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),

        # 安装 launch 文件到 share/tb4_autonav/launch
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
         
        # 安装配置文件到 share/tb4_autonav/config
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tony',
    maintainer_email='you@example.com',
    description='TurtleBot4 mapping and navigation bringup (Python package).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 我们目前不需要可执行脚本，后面如果要加节点可以在这里加
        'console_scripts': [
            # 'node_name = tb4_autonav.module_name:main_function',
            'waypoint_navigator = tb4_autonav.waypoint_navigator:main',
        ],
    },
)
