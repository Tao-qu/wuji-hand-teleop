from setuptools import setup, find_packages

package_name = 'controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Wuji Robotics',
    maintainer_email='dev@wuji.com',
    description='MANUS hand control, Tracker retargeting, and arm IK nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tracker_arm_retarget = controller.tracker_arm_retarget_node:main',
            'tianji_arm_controller = controller.tianji_arm_node:main',
            'wujihand2_controller = controller.wujihand2_node:main',
        ],
    },
)
