from setuptools import setup

setup(
    name='foot_pedal', version='0.1.0', packages=['foot_pedal'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/foot_pedal']),
        ('share/foot_pedal', ['package.xml', 'README.md']),
        ('share/foot_pedal/udev', ['udev/70-pcsensor-foot-pedal.rules']),
    ],
    install_requires=['setuptools', 'evdev>=1.6'], zip_safe=True,
    maintainer='Wuji Robotics', maintainer_email='dev@wuji.com',
    description='Standalone device-specific foot pedal event test node', license='MIT',
    entry_points={'console_scripts': ['pedal = foot_pedal.node:main']},
)
