from glob import glob
from setuptools import setup

package_name = "seer_agv_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ybbb",
    maintainer_email="user@example.com",
    description="ROS 2 Python driver for SEER AMB AGV TCP APIs.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "seer_agv_node = seer_agv_driver.seer_agv_node:main",
            "seer_keyboard_teleop = seer_agv_driver.seer_keyboard_teleop:main",
        ],
    },
)
