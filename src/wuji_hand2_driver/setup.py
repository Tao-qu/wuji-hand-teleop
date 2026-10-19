from setuptools import find_packages, setup

setup(
    name="wuji_hand2_driver",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/wuji_hand2_driver"]),
        ("share/wuji_hand2_driver", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Teleoperation maintainers",
    maintainer_email="support@wuji.tech",
    description="ROS 2 adapter for the official Wuji Hand 2 SDK",
    license="MIT",
    entry_points={"console_scripts": [
        "wuji_hand2_driver = wuji_hand2_driver.driver_node:main",
    ]},
)
