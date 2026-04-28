"""ROS 2 voxel grid source backed by two PointCloud2 topics.

`octomap_server` publishes:
  - /octomap_point_cloud_centers   (PointCloud2)  — occupied cell centers
  - /free_cells_centers            (PointCloud2)  — free cell centers
                                                     (only if publish_free_space:=true)

We subscribe to both, on every pair-update we rebuild a SparseVoxelGrid
from the latest occupied + free clouds. UNKNOWN is implicit (any voxel
not present in either cloud).

The voxel resolution is taken from a parameter; this should match what
octomap_server is using. Origin is fixed at (0,0,0) — the world-frame
origin in your sim — but voxel indexing handles arbitrary world points.
"""
from __future__ import annotations

import threading
from typing import List, Optional, Tuple

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ...application import IVoxelGridSource
from ...domain import SparseVoxelGrid


class PointCloudVoxelGridSource(IVoxelGridSource):
    """IVoxelGridSource backed by occupied + free PointCloud2 topics."""

    def __init__(
        self,
        node: Node,
        occupied_topic: str,
        free_topic: str,
        resolution: float,
        origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._node = node
        self._resolution = resolution
        self._origin = origin

        self._occupied_points: List[Tuple[float, float, float]] = []
        self._free_points: List[Tuple[float, float, float]] = []
        self._lock = threading.Lock()
        self._frame_id: Optional[str] = None

        # OctoMap publishers default to BEST_EFFORT + VOLATILE; match that.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._occ_sub = node.create_subscription(
            PointCloud2, occupied_topic, self._on_occupied, qos)
        self._free_sub = node.create_subscription(
            PointCloud2, free_topic, self._on_free, qos)

        node.get_logger().info(
            f"PointCloudVoxelGridSource: occupied='{occupied_topic}', "
            f"free='{free_topic}', res={resolution}m"
        )

    # ------------------------------------------------------------------ callbacks
    def _on_occupied(self, msg: PointCloud2) -> None:
        pts = self._read_points(msg)
        with self._lock:
            self._occupied_points = pts
            self._frame_id = msg.header.frame_id

    def _on_free(self, msg: PointCloud2) -> None:
        pts = self._read_points(msg)
        with self._lock:
            self._free_points = pts
            if self._frame_id is None:
                self._frame_id = msg.header.frame_id

    @staticmethod
    def _read_points(msg: PointCloud2) -> List[Tuple[float, float, float]]:
        # `read_points` returns a numpy structured array; we just want xyz.
        arr = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if arr.size == 0:
            return []
        return [tuple(row) for row in np.asarray(arr, dtype=np.float64)]

    # ------------------------------------------------------------------ port impl
    def get_latest(self) -> Optional[SparseVoxelGrid]:
        with self._lock:
            occ = list(self._occupied_points)
            free = list(self._free_points)
        if not occ and not free:
            return None
        grid = SparseVoxelGrid(resolution=self._resolution, origin=self._origin)
        grid.add_occupied_points(occ)
        grid.add_free_points(free)
        return grid

    @property
    def frame_id(self) -> Optional[str]:
        return self._frame_id
