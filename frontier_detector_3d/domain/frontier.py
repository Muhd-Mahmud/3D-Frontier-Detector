"""Domain types describing detected frontiers.

A frontier voxel is a single FREE voxel that has at least one UNKNOWN
neighbor. A frontier cluster is a connected group of such voxels — that
is the actual unit a planner cares about, since exploration goals are
regions, not single cells.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from .voxel import VoxelCoord


@dataclass(frozen=True)
class FrontierVoxel:
    """A FREE voxel touching at least one UNKNOWN neighbor.

    `unknown_neighbor_count` is a cheap local information-gain proxy:
    voxels with more unknown neighbors point toward bigger pockets of
    unexplored space.
    """

    coord: VoxelCoord
    unknown_neighbor_count: int


@dataclass(frozen=True)
class FrontierCluster:
    """A spatially-connected group of frontier voxels.

    The centroid (in world coordinates, metres) is the goal candidate
    that gets handed up to the planner. `information_gain` is the sum
    of unknown-neighbor counts across the cluster — a coarse-but-useful
    measure of how much new map we expect to see by going there.
    """

    id: int
    voxels: Tuple[FrontierVoxel, ...]
    centroid_world: Tuple[float, float, float]
    information_gain: int

    @property
    def size(self) -> int:
        return len(self.voxels)


@dataclass
class FrontierDetectionResult:
    """Container returned by the detector for one map snapshot."""

    clusters: List[FrontierCluster] = field(default_factory=list)
    total_frontier_voxels: int = 0
    map_resolution: float = 0.0

    def top_k_by_gain(self, k: int) -> List[FrontierCluster]:
        return sorted(self.clusters, key=lambda c: c.information_gain, reverse=True)[:k]
