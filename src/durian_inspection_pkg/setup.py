from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'durian_inspection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=[
        'inspection_server',
        'navigator',
        'gui',
        'map_publisher',
        'vision_node',
        'depth',
        'bridge_node'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/durian_inspection_pkg']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johny',
    maintainer_email='johny@todo.todo',
    description='Durian Inspection Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vision_node = vision_node:main',
            # 'video_publisher = durian_inspection_pkg.video_publisher:main',
            'inspection_server = inspection_server:main',
            'depth_node = depth:main',
            'bridge_node = bridge_node:main',
            'map_publisher = map_publisher:main',
            'navigator = navigator:main',
            'gui = gui:main'
        ],
    },
)