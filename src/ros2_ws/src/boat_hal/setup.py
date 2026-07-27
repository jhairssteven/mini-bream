from glob import glob

from setuptools import find_packages, setup

package_name = "boat_hal"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="steven",
    maintainer_email="71862429+jhairssteven@users.noreply.github.com",
    description="Hardware abstraction layer for BlueBoat simulation and field hardware.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "topic_contract_node = boat_hal.topic_contract_node:main",
        ],
    },
)
