from glob import glob

from setuptools import setup

package_name = "aruco_mocap"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Terry Yang",
    maintainer_email="terry.yang.apply@gmail.com",
    description="D435i ArUco + PnP pose, checked against motion capture ground truth.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "aruco_pose = aruco_mocap.aruco_node:main",
            "mocap_gt = aruco_mocap.mocap_gt_node:main",
            "fake_mocap = aruco_mocap.fake_mocap:main",
        ],
    },
)
