from setuptools import setup
from glob import glob
import os

package_name = "my_arm_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Manel Puig",
    maintainer_email="puigmanel@gmail.com",
    description="ROS 2 serial driver for a servo-based educational robot arm",
    license="MIT",
    #tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge_node = my_arm_driver.serial_bridge_node:main",
            "serial_trajectory_bridge_node = my_arm_driver.serial_trajectory_bridge_node:main",
            "test_joint_trajectory_node = my_arm_driver.test_joint_trajectory_node:main",
            "send_joint_target_node = my_arm_driver.send_joint_target_node:main",
            'send_joint_trajectory_node = my_arm_driver.send_joint_trajectory_node:main',
        ],
    },
)