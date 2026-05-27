from setuptools import setup

package_name = 'my_arm_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manel Puig',
    maintainer_email='manel.puig@ub.edu',
    description='Node bridge entre ROS2 i Arduino via Serial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = my_robot_arm_driver.serial_bridge_node:main'
        ],
    },
)
