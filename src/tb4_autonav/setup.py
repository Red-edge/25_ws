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
        'console_scripts': [
            # 'node_name = tb4_autonav.module_name:main_function',
            'waypoint_navigator = tb4_autonav.waypoint_navigator:main',
            'task_navigator = tb4_autonav.task_navigator:main',
            'task_combined_navigator = tb4_autonav.task_combined_navigator:main',
            'traffic_detector_node = tb4_autonav.traffic_detector_node:main',
            'pan_tilt_sweep = tb4_autonav.pan_tilt_sweep:main',
            'yolo_detector_node = tb4_autonav.yolo_detector_node:main',
            'traffic_detector_yolo = tb4_autonav.traffic_detector_yolo:main',
            'image_recorder = tb4_autonav.image_recorder:main',
        ],
    },
)
