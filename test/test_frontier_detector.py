"""Unit tests for find_frontier_voxels, cluster_frontiers, and FrontierDetector.

Each test builds a tiny hand-crafted grid where the right answer is obvious
by inspection, then asserts the algorithm matches.
"""
import pytest

from frontier_detector_3d.domain import (
    FrontierDetector,
    SparseVoxelGrid,
    VoxelCoord,
    cluster_frontiers,
    find_frontier_voxels,
)


def _make_free_strip(grid: SparseVoxelGrid, length: int, j: int = 0, k: int = 0) -> None:
    """Mark voxels (0..length-1, j, k) as FREE."""
    for i in range(length):
        grid.mark_free(VoxelCoord(i, j, k))


class TestFindFrontierVoxels:
    def test_isolated_free_voxel_is_a_frontier(self) -> None:
        # Single free voxel surrounded by unknown — all 6 neighbors unknown.
        grid = SparseVoxelGrid(resolution=0.1)
        grid.mark_free(VoxelCoord(0, 0, 0))
        frontiers = find_frontier_voxels(grid)
        assert len(frontiers) == 1
        assert frontiers[0].coord == VoxelCoord(0, 0, 0)
        assert frontiers[0].unknown_neighbor_count == 6

    def test_interior_of_free_volume_is_not_a_frontier(self) -> None:
        # A 3x3x3 cube of free voxels: the center voxel has all-free neighbors.
        grid = SparseVoxelGrid(resolution=0.1)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    grid.mark_free(VoxelCoord(i, j, k))
        frontiers = find_frontier_voxels(grid)
        center = VoxelCoord(1, 1, 1)
        # The center should have 0 unknown neighbors → not a frontier.
        for fv in frontiers:
            assert fv.coord != center

    def test_strip_endpoints_are_frontiers(self) -> None:
        # Free strip along x-axis of length 5.
        # Each end voxel has 5 unknown neighbors (all but the inward one).
        # Each middle voxel has 4 unknown neighbors (all but ±x).
        grid = SparseVoxelGrid(resolution=0.1)
        _make_free_strip(grid, 5)
        frontiers = find_frontier_voxels(grid)
        assert len(frontiers) == 5  # every cell touches unknown
        endpoint_counts = sorted(fv.unknown_neighbor_count for fv in frontiers)
        assert endpoint_counts == [4, 4, 4, 5, 5]

    def test_occupied_voxels_are_never_frontiers(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        grid.mark_occupied(VoxelCoord(0, 0, 0))
        frontiers = find_frontier_voxels(grid)
        assert frontiers == []

    def test_free_voxel_walled_off_by_occupied_is_not_frontier(self) -> None:
        # Free voxel at center, all 6 neighbors OCCUPIED → no UNKNOWN nearby.
        grid = SparseVoxelGrid(resolution=0.1)
        center = VoxelCoord(0, 0, 0)
        grid.mark_free(center)
        for n in center.neighbors_6():
            grid.mark_occupied(n)
        frontiers = find_frontier_voxels(grid)
        assert frontiers == []


class TestClusterFrontiers:
    def test_empty_input_returns_empty(self) -> None:
        assert cluster_frontiers([]) == []

    def test_two_disjoint_groups_yield_two_clusters(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        # Group A: free strip at y=0, length 5.
        _make_free_strip(grid, 5, j=0)
        # Group B: free strip at y=10, length 5 (far away — not 26-connected).
        _make_free_strip(grid, 5, j=10)
        frontiers = find_frontier_voxels(grid)
        clusters = cluster_frontiers(frontiers, min_cluster_size=1, grid=grid)
        assert len(clusters) == 2
        # Both clusters should have 5 voxels.
        assert sorted(c.size for c in clusters) == [5, 5]

    def test_min_cluster_size_filters_noise(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        # One big group of 5, one small singleton — far apart.
        _make_free_strip(grid, 5, j=0)
        grid.mark_free(VoxelCoord(0, 20, 0))  # singleton
        frontiers = find_frontier_voxels(grid)
        clusters = cluster_frontiers(frontiers, min_cluster_size=3, grid=grid)
        assert len(clusters) == 1
        assert clusters[0].size == 5

    def test_clusters_have_unique_ids(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        _make_free_strip(grid, 4, j=0)
        _make_free_strip(grid, 4, j=10)
        _make_free_strip(grid, 4, j=20)
        frontiers = find_frontier_voxels(grid)
        clusters = cluster_frontiers(frontiers, min_cluster_size=1, grid=grid)
        ids = [c.id for c in clusters]
        assert len(ids) == len(set(ids))

    def test_centroid_is_in_world_coordinates(self) -> None:
        # Single isolated free voxel at index (10, 10, 10), resolution 0.5,
        # origin (1, 2, 3). World center = (1 + 10.5*0.5, 2 + 10.5*0.5, 3 + 10.5*0.5)
        grid = SparseVoxelGrid(resolution=0.5, origin=(1.0, 2.0, 3.0))
        grid.mark_free(VoxelCoord(10, 10, 10))
        frontiers = find_frontier_voxels(grid)
        clusters = cluster_frontiers(frontiers, min_cluster_size=1, grid=grid)
        assert len(clusters) == 1
        cx, cy, cz = clusters[0].centroid_world
        assert cx == pytest.approx(1 + 10.5 * 0.5)
        assert cy == pytest.approx(2 + 10.5 * 0.5)
        assert cz == pytest.approx(3 + 10.5 * 0.5)

    def test_information_gain_is_sum_of_unknown_neighbors(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        # Single isolated free voxel: 6 unknown neighbors.
        grid.mark_free(VoxelCoord(0, 0, 0))
        frontiers = find_frontier_voxels(grid)
        clusters = cluster_frontiers(frontiers, min_cluster_size=1, grid=grid)
        assert clusters[0].information_gain == 6


class TestFrontierDetector:
    def test_full_pipeline_on_strip(self) -> None:
        grid = SparseVoxelGrid(resolution=0.2)
        _make_free_strip(grid, 10)
        result = FrontierDetector(min_cluster_size=3).detect(grid)
        assert result.total_frontier_voxels == 10
        assert len(result.clusters) == 1
        assert result.map_resolution == 0.2

    def test_top_k_by_gain_orders_descending(self) -> None:
        grid = SparseVoxelGrid(resolution=0.1)
        # Two clusters of different sizes — bigger one has more total info gain.
        _make_free_strip(grid, 10, j=0)
        _make_free_strip(grid, 4, j=20)
        result = FrontierDetector(min_cluster_size=1).detect(grid)
        top = result.top_k_by_gain(2)
        assert len(top) == 2
        assert top[0].information_gain >= top[1].information_gain

    def test_invalid_min_cluster_size_raises(self) -> None:
        with pytest.raises(ValueError):
            FrontierDetector(min_cluster_size=0)
