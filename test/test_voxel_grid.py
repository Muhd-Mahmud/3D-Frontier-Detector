"""Unit tests for the pure-domain SparseVoxelGrid.

No ROS context, no rclpy.init() — these run in plain pytest.
"""
import pytest

from frontier_detector_3d.domain import SparseVoxelGrid, VoxelCoord, VoxelState


class TestSparseVoxelGrid:
    def test_unknown_is_implicit_default(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        assert grid.is_unknown(VoxelCoord(0, 0, 0))
        assert grid.state_at(VoxelCoord(5, 5, 5)) is VoxelState.UNKNOWN

    def test_mark_free_then_query(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        c = VoxelCoord(1, 2, 3)
        grid.mark_free(c)
        assert grid.is_free(c)
        assert not grid.is_occupied(c)
        assert not grid.is_unknown(c)

    def test_mark_occupied_overrides_free(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        c = VoxelCoord(1, 2, 3)
        grid.mark_free(c)
        grid.mark_occupied(c)
        assert grid.is_occupied(c)
        assert not grid.is_free(c)

    def test_setting_unknown_removes_from_storage(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        c = VoxelCoord(1, 2, 3)
        grid.mark_free(c)
        assert len(grid) == 1
        grid.set_state(c, VoxelState.UNKNOWN)
        assert len(grid) == 0
        assert grid.is_unknown(c)

    def test_resolution_must_be_positive(self) -> None:
        with pytest.raises(ValueError):
            SparseVoxelGrid(resolution=0.0)
        with pytest.raises(ValueError):
            SparseVoxelGrid(resolution=-1.0)

    def test_world_to_voxel_round_trip(self) -> None:
        grid = SparseVoxelGrid(resolution=0.5, origin=(1.0, 2.0, 3.0))
        coord = VoxelCoord.from_world((1.75, 2.75, 3.75), 0.5, (1.0, 2.0, 3.0))
        # 1.75 - 1.0 = 0.75 → floor(0.75 / 0.5) = 1
        assert coord == VoxelCoord(1, 1, 1)
        wx, wy, wz = grid.voxel_to_world(coord)
        assert wx == pytest.approx(1.75)
        assert wy == pytest.approx(2.75)
        assert wz == pytest.approx(3.75)

    def test_add_free_points_buckets_into_voxels(self) -> None:
        grid = SparseVoxelGrid(resolution=1.0)
        # Three points all inside the (0,0,0) voxel:
        grid.add_free_points([(0.1, 0.2, 0.3), (0.5, 0.5, 0.5), (0.9, 0.1, 0.4)])
        assert len(grid) == 1
        assert grid.is_free(VoxelCoord(0, 0, 0))

    def test_free_voxels_iterator_excludes_occupied(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        grid.mark_free(VoxelCoord(0, 0, 0))
        grid.mark_free(VoxelCoord(1, 0, 0))
        grid.mark_occupied(VoxelCoord(2, 0, 0))
        assert sorted(grid.free_voxels(), key=lambda c: c.i) == [
            VoxelCoord(0, 0, 0), VoxelCoord(1, 0, 0)
        ]


class TestVoxelCoord:
    def test_neighbors_6_count(self) -> None:
        c = VoxelCoord(0, 0, 0)
        assert len(list(c.neighbors_6())) == 6

    def test_neighbors_26_count(self) -> None:
        c = VoxelCoord(0, 0, 0)
        assert len(list(c.neighbors_26())) == 26

    def test_neighbors_6_correctness(self) -> None:
        c = VoxelCoord(5, 5, 5)
        ns = set(c.neighbors_6())
        assert VoxelCoord(6, 5, 5) in ns
        assert VoxelCoord(5, 6, 5) in ns
        assert VoxelCoord(5, 5, 6) in ns
        assert VoxelCoord(4, 5, 5) in ns
        # Diagonal should NOT be in 6-connectivity:
        assert VoxelCoord(6, 6, 5) not in ns

    def test_hashable(self) -> None:
        s = {VoxelCoord(1, 2, 3), VoxelCoord(1, 2, 3), VoxelCoord(4, 5, 6)}
        assert len(s) == 2
