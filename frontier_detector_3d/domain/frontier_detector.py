"""3D frontier detection algorithm.

Two responsibilities, both pure:

1. `find_frontier_voxels` — single pass over all FREE voxels in the grid;
   a voxel is a frontier if at least one of its 6 face-neighbors is UNKNOWN.

2. `cluster_frontiers` — connected-components over the frontier voxel set
   using BFS on 26-connectivity. Each cluster becomes a FrontierCluster
   with a world-frame centroid and an information-gain score.

The detector wires them together and applies a minimum-cluster-size filter
to suppress noise (single isolated frontier voxels caused by sensor
fluctuations).
"""
from __future__ import annotations

from collections import deque
from typing import Dict, List, Set

from .frontier import FrontierCluster, FrontierDetectionResult, FrontierVoxel
from .voxel import VoxelCoord
from .voxel_grid import SparseVoxelGrid


def find_frontier_voxels(grid: SparseVoxelGrid) -> List[FrontierVoxel]:
    """Return every FREE voxel in `grid` that touches at least one UNKNOWN cell.

    Complexity: O(F) where F = number of FREE voxels (each checks 6 neighbors).
    """
    result: List[FrontierVoxel] = []
    for coord in grid.free_voxels():
        unknown_count = sum(1 for n in coord.neighbors_6() if grid.is_unknown(n))
        if unknown_count > 0:
            result.append(FrontierVoxel(coord=coord, unknown_neighbor_count=unknown_count))
    return result


def cluster_frontiers(
    frontier_voxels: List[FrontierVoxel],
    min_cluster_size: int = 3,
    grid: SparseVoxelGrid | None = None,
) -> List[FrontierCluster]:
    """Group connected frontier voxels using 26-connectivity BFS.

    Clusters smaller than `min_cluster_size` are discarded — these are
    almost always sensor noise rather than real exploration goals.

    `grid` is needed only to compute world-frame centroids; if omitted,
    centroids will be in voxel-index space (useful for unit tests).
    """
    if not frontier_voxels:
        return []

    by_coord: Dict[VoxelCoord, FrontierVoxel] = {fv.coord: fv for fv in frontier_voxels}
    visited: Set[VoxelCoord] = set()
    clusters: List[FrontierCluster] = []
    next_id = 0

    for seed_coord in by_coord:
        if seed_coord in visited:
            continue

        # BFS over the frontier-voxel set, 26-connectivity.
        component: List[FrontierVoxel] = []
        queue: deque[VoxelCoord] = deque([seed_coord])
        visited.add(seed_coord)

        while queue:
            current = queue.popleft()
            component.append(by_coord[current])
            for neighbor in current.neighbors_26():
                if neighbor in by_coord and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        if len(component) < min_cluster_size:
            continue

        # Centroid in world frame (or voxel-index frame if no grid given).
        if grid is not None:
            xs = ys = zs = 0.0
            for fv in component:
                wx, wy, wz = grid.voxel_to_world(fv.coord)
                xs += wx; ys += wy; zs += wz
            n = len(component)
            centroid = (xs / n, ys / n, zs / n)
        else:
            n = len(component)
            cx = sum(fv.coord.i for fv in component) / n
            cy = sum(fv.coord.j for fv in component) / n
            cz = sum(fv.coord.k for fv in component) / n
            centroid = (cx, cy, cz)

        info_gain = sum(fv.unknown_neighbor_count for fv in component)

        clusters.append(FrontierCluster(
            id=next_id,
            voxels=tuple(component),
            centroid_world=centroid,
            information_gain=info_gain,
        ))
        next_id += 1

    return clusters


class FrontierDetector:
    """Stateless detector — instances are cheap, parameters are configurable."""

    def __init__(self, min_cluster_size: int = 3) -> None:
        if min_cluster_size < 1:
            raise ValueError("min_cluster_size must be >= 1")
        self._min_cluster_size = min_cluster_size

    def detect(self, grid: SparseVoxelGrid) -> FrontierDetectionResult:
        frontier_voxels = find_frontier_voxels(grid)
        clusters = cluster_frontiers(
            frontier_voxels,
            min_cluster_size=self._min_cluster_size,
            grid=grid,
        )
        return FrontierDetectionResult(
            clusters=clusters,
            total_frontier_voxels=len(frontier_voxels),
            map_resolution=grid.resolution,
        )
