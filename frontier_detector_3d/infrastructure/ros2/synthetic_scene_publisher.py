"""Synthetic scene publisher for a self-contained demo.

Builds a hollow box-shaped "room" in voxel space, where:

  - the room walls are OCCUPIED voxels
  - a partial volume inside the room (a sphere expanding from one
    corner) has been "explored" — those interior voxels are FREE
  - everything else is implicit UNKNOWN

This produces a lovely curved frontier surface where the explored
sphere bumps up against unexplored interior. Run the detector against
this and you see clean clustered frontiers without needing a drone or
a real OctoMap. Perfect for portfolio screenshots.

Two PointCloud2 topics published:
    /octomap_point_cloud_centers    (occupied)
    /free_cells_centers             (free)

— same topic names octomap_server uses, so the detector wires up
identically in the real and synthetic cases.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _make_room_walls(size_x: int, size_y: int, size_z: int, resolution: float
                     ) -> List[Tuple[float, float, float]]:
    """Return world-frame points on the 6 walls of a hollow box."""
    pts: List[Tuple[float, float, float]] = []
    for i in range(size_x):
        for j in range(size_y):
            for k in range(size_z):
                on_wall = (
                    i == 0 or i == size_x - 1 or
                    j == 0 or j == size_y - 1 or
                    k == 0 or k == size_z - 1
                )
                if on_wall:
                    pts.append(((i + 0.5) * resolution,
                                (j + 0.5) * resolution,
                                (k + 0.5) * resolution))
    return pts


def _make_explored_sphere(center: Tuple[int, int, int], radius: int,
                          size_x: int, size_y: int, size_z: int,
                          resolution: float
                          ) -> List[Tuple[float, float, float]]:
    """Return world-frame points for interior voxels inside a sphere
    (excluding the wall layer)."""
    cx, cy, cz = center
    pts: List[Tuple[float, float, float]] = []
    for i in range(1, size_x - 1):
        for j in range(1, size_y - 1):
            for k in range(1, size_z - 1):
                if (i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2 <= radius ** 2:
                    pts.append(((i + 0.5) * resolution,
                                (j + 0.5) * resolution,
                                (k + 0.5) * resolution))
    return pts


class SyntheticScenePublisher(Node):
    """Publishes occupied + free PointCloud2 every tick.

    The 'explored' sphere grows over time so you can watch the frontier
    advance into unexplored space — exactly the dynamic the real system
    will produce when a UAV is flying.
    """

    def __init__(self) -> None:
        super().__init__("synthetic_scene_publisher")

        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("room_size_x", 30)
        self.declare_parameter("room_size_y", 30)
        self.declare_parameter("room_size_z", 12)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("expand_per_tick", 0.5)
        self.declare_parameter("initial_radius", 4.0)

        self._res = self.get_parameter("resolution").value
        self._sx = self.get_parameter("room_size_x").value
        self._sy = self.get_parameter("room_size_y").value
        self._sz = self.get_parameter("room_size_z").value
        self._frame_id = self.get_parameter("frame_id").value
        rate = self.get_parameter("publish_rate_hz").value
        self._expand = self.get_parameter("expand_per_tick").value
        self._radius = float(self.get_parameter("initial_radius").value)

        self._occ_pub = self.create_publisher(
            PointCloud2, "octomap_point_cloud_centers", 10)
        self._free_pub = self.create_publisher(
            PointCloud2, "free_cells_centers", 10)

        # Walls are static; pre-compute once.
        self._wall_pts = _make_room_walls(self._sx, self._sy, self._sz, self._res)

        # Sphere center: lower corner of the room so the frontier always
        # has somewhere unexplored to advance into.
        self._sphere_center = (5, 5, 3)

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Synthetic scene: {self._sx}x{self._sy}x{self._sz} room @ {self._res}m, "
            f"publishing on /octomap_point_cloud_centers & /free_cells_centers"
        )

    def _tick(self) -> None:
        # Grow the explored region.
        self._radius += self._expand
        max_dim = max(self._sx, self._sy, self._sz)
        if self._radius > max_dim * 1.5:
            # Reset to initial — gives a fresh demo loop.
            self._radius = float(self.get_parameter("initial_radius").value)

        free_pts = _make_explored_sphere(
            self._sphere_center, int(self._radius),
            self._sx, self._sy, self._sz, self._res
        )

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._frame_id)

        self._occ_pub.publish(_xyz_cloud(self._wall_pts, header))
        self._free_pub.publish(_xyz_cloud(free_pts, header))

        self.get_logger().debug(
            f"Tick: walls={len(self._wall_pts)} free={len(free_pts)} radius={self._radius:.1f}"
        )


def _xyz_cloud(points: List[Tuple[float, float, float]], header: Header) -> PointCloud2:
    arr = np.asarray(points, dtype=np.float32) if points else np.zeros((0, 3), dtype=np.float32)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return point_cloud2.create_cloud(header, fields, arr)


def main() -> None:
    rclpy.init()
    node = SyntheticScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
