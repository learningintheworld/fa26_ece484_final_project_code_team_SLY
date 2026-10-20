from glob import glob
from setuptools import setup

setup(
    name="sly_autonomy",
    version="0.1.0",
    packages=["sly_autonomy"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sly_autonomy"]),
        ("share/sly_autonomy", ["package.xml"]),
        ("share/sly_autonomy/launch", glob("launch/*.launch.py")),
        ("share/sly_autonomy/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team SLY",
    maintainer_email="xinweiy3@illinois.edu",
    description="Simulation-only control and stop-supervisor starter",
    license="All rights reserved; course project",
    entry_points={"console_scripts": ["sim_controller = sly_autonomy.sim_node:main"]},
)
