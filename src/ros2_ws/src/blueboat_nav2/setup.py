from setuptools import setup

package_name = "blueboat_nav2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Steven Gallego",
    maintainer_email="root@todo.todo",
    description="Nav2 planning stack for BlueBoat",
    license="MIT",
)
