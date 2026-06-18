from setuptools import find_packages
from setuptools import setup

package_name = "greenhouse_teleop_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/panther_gui_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="syedusamabinsabir",
    maintainer_email="syedusamabinsabir@example.com",
    description="Keyboard and GUI teleoperation for greenhouse Panther simulation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "panther_gui_teleop = greenhouse_teleop_control.panther_gui_teleop:main",
        ],
    },
)
