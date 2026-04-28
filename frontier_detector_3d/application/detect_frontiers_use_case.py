"""DetectFrontiersUseCase — the orchestration glue.

Pulled out of the ROS node so it can be unit-tested without spinning up
rclpy. The node just calls `execute()` on every map update.
"""
from __future__ import annotations

from typing import Optional

from ..domain import FrontierDetectionResult, FrontierDetector
from .ports import IFrontierPublisher, IVoxelGridSource


class DetectFrontiersUseCase:
    """One detection pass: pull latest grid, run detector, publish result."""

    def __init__(
        self,
        source: IVoxelGridSource,
        publisher: IFrontierPublisher,
        detector: FrontierDetector,
        frame_id: str = "map",
    ) -> None:
        self._source = source
        self._publisher = publisher
        self._detector = detector
        self._frame_id = frame_id

    def execute(self) -> Optional[FrontierDetectionResult]:
        grid = self._source.get_latest()
        if grid is None or len(grid) == 0:
            return None
        result = self._detector.detect(grid)
        self._publisher.publish(result, frame_id=self._frame_id)
        return result
