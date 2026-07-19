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
            "arm_movej_exe = my_arm_motion.arm_movej:main",
            "arm_movej_sing_exe = my_arm_motion.arm_movej_sing:main",
            "arm_movel_exe = my_arm_motion.arm_movel:main",
            "arm_movel_sing_exe = my_arm_motion.arm_movel_sing:main",
            "arm_motion_sequence_exe = my_arm_motion.arm_motion_sequence:main",
            "arm_movej_candidates_exe = my_arm_motion.arm_movej_candidates:main",
            "arm_movel_candidates_exe = my_arm_motion.arm_movel_candidates:main",
            "arm_execute_saved_exe = my_arm_motion.arm_execute_saved:main",
            "arm_test_scene_exe = my_arm_motion.arm_test_scene:main",
        ],
    },
)