from setuptools import find_packages, setup

package_name = "frontier_detector_3d"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
            ["launch/frontier_detector.launch.py", "launch/demo.launch.py"]),
        ("share/" + package_name + "/config",
            ["config/params.yaml", "config/frontiers.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Standalone 3D frontier detector for volumetric maps "
                "(clean architecture, ROS 2 jazzy).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "frontier_detector_node = "
            "frontier_detector_3d.infrastructure.ros2.frontier_detector_node:main",
            "synthetic_scene_publisher = "
            "frontier_detector_3d.infrastructure.ros2.synthetic_scene_publisher:main",
        ],
    },
)
