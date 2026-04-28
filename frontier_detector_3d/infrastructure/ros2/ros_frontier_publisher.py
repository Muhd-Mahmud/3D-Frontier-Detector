"""ROS 2 publisher for frontier detection results.

Two outputs:

  1. /frontiers/markers   visualization_msgs/MarkerArray
       Spheres at each cluster centroid, sized by information gain,
       color-graded so the best frontier is visually obvious in RViz.

  2. /frontiers/goals     geometry_msgs/PoseArray
       Same centroids in machine-readable form for the planner above.

We always publish a DELETEALL marker first so stale clusters from the
previous tick don't linger in RViz.
"""
from __future__ import annotations

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from ...application import IFrontierPublisher
from ...domain import FrontierDetectionResult


class RosFrontierPublisher(IFrontierPublisher):
    """Concrete publisher that writes MarkerArray + PoseArray to ROS topics."""

    def __init__(
        self,
        node: Node,
        markers_topic: str = "frontiers/markers",
        goals_topic: str = "frontiers/goals",
        marker_namespace: str = "frontiers",
        min_marker_radius: float = 0.15,
        max_marker_radius: float = 0.6,
    ) -> None:
        self._node = node
        self._ns = marker_namespace
        self._min_r = min_marker_radius
        self._max_r = max_marker_radius
        self._marker_pub = node.create_publisher(MarkerArray, markers_topic, 10)
        self._goal_pub = node.create_publisher(PoseArray, goals_topic, 10)

    # ------------------------------------------------------------------ port impl
    def publish(self, result: FrontierDetectionResult, frame_id: str) -> None:
        stamp = self._node.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=frame_id)

        self._marker_pub.publish(self._build_marker_array(result, header))
        self._goal_pub.publish(self._build_pose_array(result, header))

    # ------------------------------------------------------------------ helpers
    def _build_marker_array(self, result: FrontierDetectionResult, header: Header) -> MarkerArray:
        markers = MarkerArray()

        # Wipe previous-tick markers so deleted clusters disappear cleanly.
        clear = Marker()
        clear.header = header
        clear.ns = self._ns
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if not result.clusters:
            return markers

        max_gain = max(c.information_gain for c in result.clusters)
        max_gain = max(max_gain, 1)  # avoid div-by-zero

        for cluster in result.clusters:
            m = Marker()
            m.header = header
            m.ns = self._ns
            m.id = cluster.id
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            cx, cy, cz = cluster.centroid_world
            m.pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
            m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # Radius scales with normalized info gain.
            scale = self._min_r + (self._max_r - self._min_r) * (cluster.information_gain / max_gain)
            m.scale.x = m.scale.y = m.scale.z = scale

            # Color graded green→red by gain rank: best gets pure green, worst red.
            normalized = cluster.information_gain / max_gain
            m.color = ColorRGBA(r=float(1.0 - normalized), g=float(normalized), b=0.2, a=0.85)
            m.lifetime.sec = 0  # cleared by DELETEALL on next tick

            markers.markers.append(m)

        return markers

    @staticmethod
    def _build_pose_array(result: FrontierDetectionResult, header: Header) -> PoseArray:
        pa = PoseArray()
        pa.header = header
        # Highest information gain first — planner can just pop poses[0].
        for cluster in sorted(result.clusters, key=lambda c: c.information_gain, reverse=True):
            pose = Pose()
            cx, cy, cz = cluster.centroid_world
            pose.position = Point(x=float(cx), y=float(cy), z=float(cz))
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pa.poses.append(pose)
        return pa
