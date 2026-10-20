from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'tianji_output'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('tianji_output/config/*.MvKDCfg') + glob('tianji_output/config/*.yaml') + glob('tianji_output/config/*.yaml.template')),
    ],
    package_data={
        package_name: [
            '_internal/lib/*.so',  # Linux shared libraries
            '_internal/lib/*.dll', # Windows DLLs (if needed)
            'config/*.MvKDCfg',    # Robot configuration files
        ],
    },
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'PyYAML',
    ],
    zip_safe=False,  # Must be False because of shared libraries
    maintainer='wuji',
    maintainer_email='your_email@example.com',
    description='Tianji arm hardware interface library - exports TianjiArmController and TianjiChestDriver',
    license='MIT',
)
