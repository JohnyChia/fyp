from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'durian_inspection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'vision_node = durian_inspection_pkg.vision_node:main',
            'video_publisher = durian_inspection_pkg.video_publisher:main',
            'inspection_server = durian_inspection_pkg.inspection_server:main',
        ],
    },
)