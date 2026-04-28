"""Domain value objects for voxel-based reasoning.

Pure Python. Zero ROS / numpy dependency at this level so the algorithm
is trivially unit-testable and reusable.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterator, Tuple


class VoxelState(Enum):
    """Three-way classification of a voxel given the current map.

    The boundary we care about for exploration is FREE↔UNKNOWN; OCCUPIED
    voxels are a hard wall and never frontiers.
    """

    FREE = "free"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class VoxelCoord:
    """Discrete 3D voxel index. Frozen so it can be a dict / set key."""

    i: int
    j: int
    k: int

    def neighbors_6(self) -> Iterator["VoxelCoord"]:
        """Yield the 6 face-adjacent neighbors."""
        for di, dj, dk in (
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1),
        ):
            yield VoxelCoord(self.i + di, self.j + dj, self.k + dk)

    def neighbors_26(self) -> Iterator["VoxelCoord"]:
        """Yield the 26 face/edge/corner-adjacent neighbors."""
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if di == 0 and dj == 0 and dk == 0:
                        continue
                    yield VoxelCoord(self.i + di, self.j + dj, self.k + dk)

    def to_world(self, resolution: float, origin: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Convert this voxel index to a world-frame point at its center."""
        ox, oy, oz = origin
        return (
            ox + (self.i + 0.5) * resolution,
            oy + (self.j + 0.5) * resolution,
            oz + (self.k + 0.5) * resolution,
        )

    @staticmethod
    def from_world(point: Tuple[float, float, float], resolution: float,
                   origin: Tuple[float, float, float]) -> "VoxelCoord":
        """Inverse of to_world. Floors to the containing voxel."""
        x, y, z = point
        ox, oy, oz = origin
        return VoxelCoord(
            int((x - ox) // resolution),
            int((y - oy) // resolution),
            int((z - oz) // resolution),
        )
