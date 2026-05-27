from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony.chamba.05@gmail.com',
    description='Voice-LLM-ROS2 pipeline for robot manipulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal= my_robot_pipeline.goal:main',
            'color_detector = my_robot_pipeline.color_detector:main',
            'voice_node = my_robot_pipeline.voice_node:main',
            'llm_node = my_robot_pipeline.llm_node:main',
            'pipeline_node = my_robot_pipeline.pipeline_node:main',
            'get_pixels = my_robot_pipeline.get_pixels:main',
            'llmnode_experiment = my_robot_pipeline.llm_experiment_node:main',
        ],
    },
)
