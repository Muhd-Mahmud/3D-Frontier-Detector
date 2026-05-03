# `3D_frontier_detector`

A standalone, clean-architecture **3D frontier detector** for ROS 2 jazzy.

In autonomous exploration, a *frontier* is the boundary between known free
space and unknown space ; the place a robot should fly to next if it wants
to learn the most new geometry per unit time. This package detects 3D
frontiers in OctoMap-compatible voxel data, clusters them, ranks them by
information gain, and publishes goal candidates a planner can consume.

It runs out of the box with a built-in synthetic scene — no drone, no
SLAM, no octomap_server required. One launch file, one RViz window,
frontiers light up.

---

## Why this exists

Most public frontier detectors are 2D occupancy-grid only. UAV exploration
in unknown 3D environments needs the volumetric version: a free voxel that
has at least one unknown 6-neighbor is a frontier voxel. Group those into
connected components and you get exploration goals — exactly what feeds
the planner above.

This package is built as a **clean-architecture** ROS 2 package — the
algorithm is a pure-Python domain layer with zero ROS imports, the use
case sits in an application layer talking only to abstract ports, and
the ROS 2 adapters are in an isolated infrastructure layer. The whole
algorithm is unit-tested without spinning up rclpy.

---

## Demo (no hardware needed)

```bash
# In your colcon workspace:
cd ~/ros2_ws/src
git clone <this-repo>
cd ..
colcon build --symlink-install --packages-select frontier_detector_3d
source install/setup.bash

# Self-contained demo with synthetic scene + RViz:
ros2 launch frontier_detector_3d demo.launch.py
```

You'll see:

- **grey voxel boxes** — the walls of a synthetic 30×30×12 room (occupied)
- **translucent blue voxels** — an expanding sphere of explored free space
- **green→red spheres** — frontier cluster centroids, sized by information
  gain. The greenest one is the best next-goal candidate.

The explored region grows over time, so you can watch the frontier surface
recede outward — exactly what a flying UAV will produce in your real
system.

---

## Using with a real OctoMap

Configure your `octomap_server` to publish free space:

```bash
ros2 run octomap_server octomap_server_node \
  --ros-args -p publish_free_space:=true -p resolution:=0.2
```

Then launch the detector standalone:

```bash
ros2 launch frontier_detector_3d frontier_detector.launch.py
```

Adjust `config/params.yaml` so `resolution` matches your octomap_server
setting and `frame_id` matches your map frame.

### Topics

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| sub | `/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | Occupied voxel centers |
| sub | `/free_cells_centers` | `sensor_msgs/PointCloud2` | Free voxel centers |
| pub | `/frontiers/markers` | `visualization_msgs/MarkerArray` | RViz visualization |
| pub | `/frontiers/goals` | `geometry_msgs/PoseArray` | Ranked goal candidates (highest info gain first) |

---

## Algorithm

Two pure functions in `frontier_detector_3d/domain/frontier_detector.py`:

**`find_frontier_voxels(grid)`** — single pass over every FREE voxel; a
voxel is a frontier if at least one of its 6 face-neighbors is UNKNOWN.
Records the unknown-neighbor count as a local information-gain proxy.
Complexity: O(F) where F is the number of free voxels.

**`cluster_frontiers(voxels)`** — BFS-based connected-components labeling
on 26-connectivity. Drops clusters smaller than `min_cluster_size` to
suppress sensor noise. Returns each cluster's world-frame centroid (the
goal candidate) and a summed information-gain score.

The detector wires them together. That's it. ~100 lines of pure Python,
no numpy required, fully unit-testable.

---

## Architecture

```
frontier_detector_3d/
├── domain/                          # pure Python — no ROS, no numpy
│   ├── voxel.py                     # VoxelCoord, VoxelState
│   ├── voxel_grid.py                # SparseVoxelGrid
│   ├── frontier.py                  # FrontierVoxel, FrontierCluster
│   └── frontier_detector.py         # the algorithm
├── application/                     # orchestration — abstract ports only
│   ├── ports.py                     # IVoxelGridSource, IFrontierPublisher
│   └── detect_frontiers_use_case.py
└── infrastructure/ros2/             # ROS 2 adapters — the only ROS-aware layer
    ├── pointcloud_voxel_source.py   # subscribes to two PointCloud2 topics
    ├── ros_frontier_publisher.py    # publishes MarkerArray + PoseArray
    ├── synthetic_scene_publisher.py # demo scene generator
    └── frontier_detector_node.py    # composition root
```

Dependencies flow inward only: infrastructure depends on application,
application depends on domain, domain depends on nothing. This is what
makes the algorithm trivially unit-testable and the ROS layer trivially
swappable (e.g. for an octomap-binary-message adapter, or a non-ROS
deployment).

---

## Tests

```bash
cd ~/ros2_ws/src/frontier_detector_3d
python -m pytest test/ -v
```

26 unit tests covering the pure domain layer — voxel grid semantics,
frontier identification on hand-crafted scenes (isolated voxels, walled-
off cells, strip endpoints, full cubes), connected-components clustering,
min-size filtering, world-frame centroid computation, and information-gain
aggregation. None of them require rclpy, ROS context, or running nodes.

---

## Parameters

See `config/params.yaml`. Key ones:

| Parameter | Default | Notes |
|---|---|---|
| `resolution` | 0.2 | metres/voxel — **must match octomap_server** |
| `min_cluster_size` | 5 | voxels; smaller clusters are discarded |
| `detection_rate_hz` | 2.0 | how often to rerun detection |
| `occupied_topic` | `octomap_point_cloud_centers` | |
| `free_topic` | `free_cells_centers` | enable with `publish_free_space:=true` |
| `frame_id` | `map` | output frame for markers + poses |

---

## Roadmap / extensions

The current detector is intentionally small and focused. Natural
extensions, all of which fit cleanly into the existing layer split:

- **OctoMap binary message adapter** — replace the dual-PointCloud2 source
  with a direct `octomap_msgs/Octomap` subscriber (one new file in
  infrastructure, no domain changes).
- **Better information-gain estimate** — replace the unknown-neighbor
  count with a sphere-based unknown volume integral around the centroid.
- **Frontier cost function** — combine info gain, distance from current
  pose (read TF), and altitude preference. Pure-domain change.
- **Per-voxel observation-angle metadata** — once gimbal control lands,
  augment the voxel grid with which angles each cell has been observed
  from, and reward viewpoint diversity.

---

## License

Apache-2.0.
