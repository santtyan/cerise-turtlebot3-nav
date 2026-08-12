from setuptools import setup
from glob import glob
import os

setup(
    name='cerise_nav',
    version='0.0.1',
    packages=['cerise_nav', 'cerise_nav.rl'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cerise_nav']),
        ('share/cerise_nav', ['package.xml']),
        ('share/cerise_nav/launch', glob('../../launch/*.launch.py')),
        ('share/cerise_nav', glob('../../*.world')),
        ('share/cerise_nav', glob('../../*.model')),
    ],
    install_requires=['setuptools', 'stable-baselines3', 'gymnasium'],
    entry_points={
        'console_scripts': [
            'demand_generator = cerise_nav.demand_generator:main',
            'task_allocator = cerise_nav.task_allocator:main',
            'dataset_collector = cerise_nav.dataset_collector:main',
            'yolo_detector = cerise_nav.yolo_detector:main',
            'ekf_fusion_node = cerise_nav.ekf_fusion_node:main',
            'benchmark_detector = cerise_nav.benchmark_detector:main',
            'rl_task_allocator = cerise_nav.rl_task_allocator:main',
            'video_recorder = cerise_nav.video_recorder:main',
        ],
    },
)
