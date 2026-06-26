from glob import glob
from setuptools import setup

package_name = "offboard_nav2_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="Planner-only Nav2 bringup helpers for the offboard control workspace.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "goal_to_path = offboard_nav2_planning.goal_to_path:main",
            "odometry_tf_publisher = offboard_nav2_planning.odometry_tf_publisher:main",
            "path_2_5d_lifter = offboard_nav2_planning.path_2_5d_lifter:main",
            "save_2d_map_from_cloud = offboard_nav2_planning.save_2d_map_from_cloud:main",
            "relocalized_pose_to_tf = offboard_nav2_planning.relocalized_pose_to_tf:main",
        ],
    },
)
