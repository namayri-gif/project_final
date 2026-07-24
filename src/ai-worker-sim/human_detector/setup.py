import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'human_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/human_detector'],
        ),
        (
            'share/human_detector',
            ['package.xml'],
        ),
        (
            'share/human_detector/models',
            glob('models/*'),
        ),
        (
            'share/human_detector/launch',
            glob('launch/*.launch.py'),
        ),
    ],

    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'person_detector_node = human_detector.person_detector_node:main',
             'wave_interact = human_detector.wave_interact:main',
        ],
    },
)
