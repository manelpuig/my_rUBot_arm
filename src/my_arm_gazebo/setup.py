from setuptools import setup
from glob import glob

package_name = "my_arm_gazebo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),

        ("share/" + package_name,
         ["package.xml"]),

        ("share/" + package_name + "/launch",
         glob("launch/*.py")),

        ("share/" + package_name + "/worlds",
         glob("worlds/*")),

        ("share/" + package_name + "/config",
         glob("config/*.yaml")),
    ],

    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@todo.todo",
    description="Gazebo Sim bringup for my_arm.",
    license="Apache-2.0",
)