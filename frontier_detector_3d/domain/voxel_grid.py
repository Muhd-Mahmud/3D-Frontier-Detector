"""Sparse voxel grid — the in-memory representation the algorithm runs on.

We deliberately keep this sparse (dict-backed) instead of a dense numpy
array because real OctoMaps cover large volumes but explored cells are
a tiny fraction of total volume. A 30 m × 30 m × 10 m room at 0.1 m
resolution is 9M cells; only a few thousand are actually known.

This file contains zero ROS imports — it is pure domain.
"""
from __future__ import annotations

from typing import Dict, Iterable, Iterator, Tuple

from .voxel import VoxelCoord, VoxelState


class SparseVoxelGrid:
    """Map of VoxelCoord -> VoxelState. Missing keys are treated as UNKNOWN.

    The (resolution, origin) pair lets us round-trip between world points
    and voxel indices without losing the spatial frame.
    """

    def __init__(self, resolution: float, origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        if resolution <= 0:
            raise ValueError(f"resolution must be > 0, got {resolution}")
        self._cells: Dict[VoxelCoord, VoxelState] = {}
        self._resolution = float(resolution)
        self._origin = origin

    # ------------------------------------------------------------------ properties
    @property
    def resolution(self) -> float:
        return self._resolution

    @property
    def origin(self) -> Tuple[float, float, float]:
        return self._origin

    def __len__(self) -> int:
        return len(self._cells)

    # ------------------------------------------------------------------ mutation
    def set_state(self, coord: VoxelCoord, state: VoxelState) -> None:
        if state is VoxelState.UNKNOWN:
            # UNKNOWN is the implicit default — don't waste storage.
            self._cells.pop(coord, None)
        else:
            self._cells[coord] = state

    def mark_free(self, coord: VoxelCoord) -> None:
        self._cells[coord] = VoxelState.FREE

    def mark_occupied(self, coord: VoxelCoord) -> None:
        self._cells[coord] = VoxelState.OCCUPIED

    def add_free_points(self, points: Iterable[Tuple[float, float, float]]) -> None:
        """Mark every voxel containing one of these world-frame points as FREE."""
        for p in points:
            self.mark_free(VoxelCoord.from_world(p, self._resolution, self._origin))

    def add_occupied_points(self, points: Iterable[Tuple[float, float, float]]) -> None:
        """Mark every voxel containing one of these world-frame points as OCCUPIED."""
        for p in points:
            self.mark_occupied(VoxelCoord.from_world(p, self._resolution, self._origin))

    # ------------------------------------------------------------------ queries
    def state_at(self, coord: VoxelCoord) -> VoxelState:
        return self._cells.get(coord, VoxelState.UNKNOWN)

    def is_free(self, coord: VoxelCoord) -> bool:
        return self._cells.get(coord) is VoxelState.FREE

    def is_occupied(self, coord: VoxelCoord) -> bool:
        return self._cells.get(coord) is VoxelState.OCCUPIED

    def is_unknown(self, coord: VoxelCoord) -> bool:
        return coord not in self._cells

    def free_voxels(self) -> Iterator[VoxelCoord]:
        for coord, state in self._cells.items():
            if state is VoxelState.FREE:
                yield coord

    def voxel_to_world(self, coord: VoxelCoord) -> Tuple[float, float, float]:
        return coord.to_world(self._resolution, self._origin)
