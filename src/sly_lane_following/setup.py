from glob import glob
from setuptools import find_packages, setup

package_name = "sly_lane_following"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team SLY",
    maintainer_email="xinweiy3@illinois.edu",
    description="GEM image-domain lane selection and recorded-data evaluation",
    license="UNLICENSED",
    entry_points={
        "console_scripts": [
            "lane_node = sly_lane_following.node:main",
            "lane_offline = sly_lane_following.offline:main",
            "lane_adapter = sly_lane_following.adapter:main",
        ]
    },
)
