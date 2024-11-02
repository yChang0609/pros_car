from setuptools import find_packages, setup
from glob import glob
import os
package_name = "pros_car_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'urdf/excurate_arm'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        # ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kylin",
    maintainer_email="kylingithubdev@gmail.com",
    description="This is a pkg to control PCar.",
    license="Commercial License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_control = pros_car_py.main:main",
            "carC_reader = pros_car_py.carC_serial_reader:main",
            "carC_writer = pros_car_py.carC_serial_writer:main",
            "arm_reader = pros_car_py.arm_reader:main",
            "arm_writer = pros_car_py.arm_writer:main",
            "arm_test = pros_car_py.arm_test:main",
            "lidar_trans = pros_car_py.lidar_trans:main",
        ],
    },
)
