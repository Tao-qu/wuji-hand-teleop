from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wujihand_output'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.yaml.template')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'wuji-sdk==2026.10.12',
    ],
    zip_safe=True,
    maintainer='wuji',
    maintainer_email='your_email@example.com',
    description='Wuji Hand 2 official retargeting and ROS bridge command interface',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
