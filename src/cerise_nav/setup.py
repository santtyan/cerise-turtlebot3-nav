from setuptools import setup

setup(
    name='cerise_nav',
    version='0.0.1',
    packages=['cerise_nav'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'demand_generator = cerise_nav.demand_generator:main',
            'task_allocator = cerise_nav.task_allocator:main',
        ],
    },
)
