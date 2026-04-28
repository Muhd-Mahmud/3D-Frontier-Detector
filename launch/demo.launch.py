"""Self-contained demo launch.

Brings up:
  - synthetic_scene_publisher (fake occupied + free clouds)
  - frontier_detector_3d
  - static_transform_publisher  (world -> map, so RViz has a TF tree)
  - rviz2 with a pre-configured layout

Run with:
    ros2 launch frontier_detector_3d demo.launch.py

You should see, within 1-2 seconds, occupied wall voxels in grey, an
expanding sphere of free voxels in blue, and green->red frontier spheres
sitting on the boundary between them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("frontier_detector_3d")
    default_params = PathJoinSubstitution([pkg_share, "config", "params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "frontiers.rviz"])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to params YAML - same file is used for both nodes.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2 with the demo config.",
    )

    # Static TF: world -> map. Without this, RViz will refuse to render
    # data tagged with frame_id='map' because the frame doesn't exist
    # in the TF tree.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "map",
        ],
        output="screen",
    )

    synth = Node(
        package="frontier_detector_3d",
        executable="synthetic_scene_publisher",
        name="synthetic_scene_publisher",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    detector = Node(
        package="frontier_detector_3d",
        executable="frontier_detector_node",
        name="frontier_detector_3d",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        params_file_arg, rviz_arg,
        static_tf, synth, detector, rviz,
    ])
