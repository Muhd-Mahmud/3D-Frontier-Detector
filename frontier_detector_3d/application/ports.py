"""Application-layer ports.

These are the abstract interfaces the use case talks to. The infrastructure
layer provides concrete implementations (ROS 2 subscribers, ROS 2 publishers).
The domain knows nothing about either.

This indirection is what lets us test `DetectFrontiersUseCase` with synthetic
grids and recording publishers, with no ROS context running.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..domain import FrontierDetectionResult, SparseVoxelGrid


class IVoxelGridSource(ABC):
    """Provider of the latest fused voxel grid.

    Implementations may build the grid from PointCloud2 topics, an OctoMap
    binary message, a synthetic generator, etc. The use case doesn't care.
    """

    @abstractmethod
    def get_latest(self) -> Optional[SparseVoxelGrid]:
        """Return the most recent grid, or None if no map data has arrived yet."""


class IFrontierPublisher(ABC):
    """Sink for detection results.

    Concrete impls might push to ROS topics, write to a file, or push to a
    test buffer.
    """

    @abstractmethod
    def publish(self, result: FrontierDetectionResult, frame_id: str) -> None:
        ...
