from glob import glob
from setuptools import find_packages, setup

setup(
    name="data_collection",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/data_collection"]),
        ("share/data_collection", ["package.xml"]),
        ("share/data_collection/config", glob("config/*.yaml") + glob("config/*.yaml.template")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Wuji Robotics",
    maintainer_email="dev@wuji.com",
    description="Independent continuous LeRobot v2.1 recording",
    license="MIT",
    entry_points={"console_scripts": [
        "record = data_collection.session:main",
        "writer = data_collection.worker:main",
    ]},
)
