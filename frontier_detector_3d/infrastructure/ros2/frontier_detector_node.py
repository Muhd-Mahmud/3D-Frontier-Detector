"""FrontierDetectorNode — composition root.

Reads parameters, instantiates the source / publisher / detector / use case,
and ticks the use case on a timer. This is the only place where ROS-aware
infrastructure and the pure-domain detector come together — exactly what the
clean-architecture layering asks for.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from ...application import DetectFrontiersUseCase
from ...domain import FrontierDetector
from .pointcloud_voxel_source import PointCloudVoxelGridSource
from .ros_frontier_publisher import RosFrontierPublisher


class FrontierDetectorNode(Node):
    """ROS 2 node that runs 3D frontier detection on incoming voxel clouds."""

    def __init__(self) -> None:
        super().__init__("frontier_detector_3d")

        # ------------------------------------------------------ parameters
        self.declare_parameter("occupied_topic", "octomap_point_cloud_centers")
        self.declare_parameter("free_topic", "free_cells_centers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("min_cluster_size", 5)
        self.declare_parameter("detection_rate_hz", 2.0)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("origin_z", 0.0)

        occupied_topic = self.get_parameter("occupied_topic").value
        free_topic = self.get_parameter("free_topic").value
        frame_id = self.get_parameter("frame_id").value
        resolution = float(self.get_parameter("resolution").value)
        min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        rate = float(self.get_parameter("detection_rate_hz").value)
        origin = (
            float(self.get_parameter("origin_x").value),
            float(self.get_parameter("origin_y").value),
            float(self.get_parameter("origin_z").value),
        )

        # ------------------------------------------------------ wiring
        source = PointCloudVoxelGridSource(
            self,
            occupied_topic=occupied_topic,
            free_topic=free_topic,
            resolution=resolution,
            origin=origin,
        )
        publisher = RosFrontierPublisher(self)
        detector = FrontierDetector(min_cluster_size=min_cluster_size)
        self._use_case = DetectFrontiersUseCase(
            source=source,
            publisher=publisher,
            detector=detector,
            frame_id=frame_id,
        )

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"FrontierDetectorNode running at {rate} Hz "
            f"(resolution={resolution}m, min_cluster={min_cluster_size})"
        )

    def _tick(self) -> None:
        result = self._use_case.execute()
        if result is None:
            self.get_logger().info("Waiting for voxel cloud data...", throttle_duration_sec=5.0)
            return
        self.get_logger().info(
            f"Detected {len(result.clusters)} frontier clusters "
            f"({result.total_frontier_voxels} voxels)",
            throttle_duration_sec=2.0,
        )


def main() -> None:
    rclpy.init()
    node = FrontierDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
