from setuptools import find_packages, setup
import os
from glob import glob

package_name = "my_arm_motion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Manel Puig",
    maintainer_email="puigmanel@gmail.com",
    description="Generic MoveIt2 motion nodes for robot arms.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "puma_pose_exe = my_arm_motion.puma_pose:main",
            "move_to_pose = my_arm_motion.move_to_pose:main",
            "move_to_pose_official = my_arm_motion.move_to_pose_official:main",
        ],
    },
)