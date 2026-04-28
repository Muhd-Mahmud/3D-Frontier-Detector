"""Launch only the frontier detector node — for use with a real octomap_server.

Assumes octomap_server is publishing /octomap_point_cloud_centers and
/free_cells_centers (set publish_free_space:=true on octomap_server).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("frontier_detector_3d")
    default_params = PathJoinSubstitution([pkg_share, "config", "params.yaml"])

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to a params YAML file for the detector.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use /clock topic for time (sim mode).",
    )

    detector = Node(
        package="frontier_detector_3d",
        executable="frontier_detector_node",
        name="frontier_detector_3d",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    return LaunchDescription([params_file_arg, use_sim_time_arg, detector])
