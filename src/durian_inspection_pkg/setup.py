from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'durian_inspection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    py_modules=[
        'inspection_server',
        'gui',
        'map_publisher',
        'vision_node',
        'processing_node',
        'observation_store',
        'bridge_node',
        'collector',
        'canonical_leaf_classes',
        'checkpoint_manifest',
        'leaf_deployment_config',
        'deployment_fingerprint',
        'multi_frame_policy',
        'multi_view_inspection',
        'leaf_association',
        'unknown_outcomes',
        'experiment_spec',
        'world_gt_verifier',
        'same_input_benchmark',
        'association_calibration',
        'leaf_backend_runtime',
        'localization_stability',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/durian_inspection_pkg']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='johny',
    maintainer_email='johny@todo.todo',
    description='Durian Inspection Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vision_node = vision_node:main',
            'processing_node = processing_node:main',

            'inspection_server = inspection_server:main',
            'bridge_node = bridge_node:main',
            'map_publisher = map_publisher:main',
            'gui = gui:main',
            'collector = collector:main'
        ],
    },
)
