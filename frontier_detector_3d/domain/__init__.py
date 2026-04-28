"""Pure-domain frontier detection — no ROS, no numpy.

Anything in this subpackage can be imported and unit-tested standalone.
"""
from .frontier import FrontierCluster, FrontierDetectionResult, FrontierVoxel
from .frontier_detector import FrontierDetector, cluster_frontiers, find_frontier_voxels
from .voxel import VoxelCoord, VoxelState
from .voxel_grid import SparseVoxelGrid

__all__ = [
    "FrontierCluster",
    "FrontierDetectionResult",
    "FrontierDetector",
    "FrontierVoxel",
    "SparseVoxelGrid",
    "VoxelCoord",
    "VoxelState",
    "cluster_frontiers",
    "find_frontier_voxels",
]
