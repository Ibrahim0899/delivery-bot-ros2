from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'delivery_bot_ai'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Isaiah',
    maintainer_email='engineer@deliverybot.dev',
    description='AI/ML navigation package for DeliveryBot with Q-Learning',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_agent = delivery_bot_ai.obstacle_avoidance_agent:main',
            'reset_service = delivery_bot_ai.reset_service:main',
            'training_manager = delivery_bot_ai.training_manager:main',
        ],
    },
)
