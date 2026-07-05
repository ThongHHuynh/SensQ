# Coverage Planner

A ROS 2 package that generates and (optionally) executes **lawnmower-style coverage paths** over a SLAM-produced occupancy grid.  
The robot sweeps every reachable free-space cell while respecting a configurable safety clearance from walls and obstacles.

---

## Table of Contents

- [Overview](#overview)
- [Custom Planner Roadmap](#custom-planner-roadmap)
- [Architecture](#architecture)
- [Nodes](#nodes)
  - [coverage_node (monolithic)](#coverage_node-monolithic)
  - [map_processor_node](#map_processor_node)
  - [path_generator_node](#path_generator_node)
  - [coverage_visualizer_node](#coverage_visualizer_node)
  - [coverage_manager_node](#coverage_manager_node)
- [Topics](#topics)
- [Parameters](#parameters)
- [Launch Files](#launch-files)
- [Algorithm – How It Works](#algorithm--how-it-works)
- [How to Run](#how-to-run)
- [Improvement Ideas](#improvement-ideas)

---

## Overview

The coverage planner answers the question: *"Given a known map, how do I drive my robot over every square meter of free space?"*

It works in four stages:

1. **Map Processing** – inflate obstacles by the robot's clearance radius to produce a *safe free-space* grid.
2. **Path Generation** – decompose the safe grid into boustrophedon cells and sweep each cell.
3. **Visualization** – publish RViz markers so you can inspect waypoints before sending the robot.
4. **Execution** – (optional) send the path to Nav2 via `NavigateThroughPoses`.

---

## Custom Planner Roadmap

Goal: replace dependency-heavy coverage planning with a planner we understand, can tune, and can validate before driving the robot.

### Production-grade phases

| Phase | Goal | Done when |
|---|---|---|
| 0. Requirements | Define robot width, tool width, clearance, minimum turn radius, map frame, and success metrics | Parameters are documented and tested on one known map |
| 1. Safe map | Convert `/map` into a binary drivable grid | Walls, unknown cells, and obstacles are inflated by `clearance_m` |
| 2. Cell decomposition | Split safe space into coverage chunks | Disconnected rooms/regions become separate cells; internal obstacles become holes |
| 3. Sweep generation | Generate lawnmower passes inside each cell | Sweep spacing equals `coverage_width_m`; no waypoint is inside occupied space |
| 4. Cell ordering | Choose which cell to cover first/next | The route starts near the robot and visits all cells with reasonable transition distance |
| 5. Transition planning | Connect cells and sweep rows safely | Transitions are checked against the safe map or requested from Nav2 |
| 6. Validation | Reject unsafe paths before publishing | Every sampled point is inside safe free space and respects bounds |
| 7. Execution | Drive the path through Nav2 | Start, pause, cancel, resume, and failure handling work |
| 8. Metrics | Measure coverage quality | Publishes covered area %, missed cells, path length, and execution result |

### MVP checklist

Build the MVP in this order:

1. Keep `map_processor_node` as the source of `/coverage/safe_map`.
2. Add a shared `coverage_geometry.py` module with small data classes: `Cell`, `Ring`, `Segment`, `Waypoint`.
3. Implement `safe_map_to_cells()`:
   - use OpenCV contour hierarchy;
   - external contours become cells;
   - child contours become obstacle holes;
   - discard tiny cells/holes by area.
4. Implement `generate_sweeps(cell)`:
   - start with horizontal sweeps only;
   - intersect each sweep row with the cell polygon;
   - subtract obstacle-hole intervals;
   - create left-to-right/right-to-left segments.
5. Implement `order_segments()`:
   - start from the segment closest to the robot pose;
   - alternate direction for normal lawnmower motion;
   - jump to nearest unvisited segment only when disconnected.
6. Implement `connect_segments()`:
   - first MVP: L-shaped transitions checked against the safe grid;
   - later: ask Nav2 for transition paths between disconnected cells.
7. Implement `validate_path()`:
   - sample every `path_step_m`;
   - reject if any sample is outside safe free space;
   - warn when coverage is below target.
8. Publish:
   - `/coverage/path` as `nav_msgs/Path`;
   - `/coverage/waypoint_markers` for RViz;
   - `/coverage/metrics` later.
9. Execute only after visualization looks correct.

MVP acceptance criteria:

- Handles one room with internal obstacles.
- Handles two disconnected free-space regions.
- Path never crosses occupied/unknown cells in `/coverage/safe_map`.
- Sweep spacing is predictable from `coverage_width_m`.
- RViz visualization is readable before execution.

---

## Architecture

The package ships **two architectures** selectable via launch file:

| Launch file | Architecture | Nodes started |
|---|---|---|
| `coverage.launch.py` | **Monolithic** – single node does everything | `coverage_node` |
| `zigzag.launch.py` | **Modular pipeline** – each stage is a separate node | `map_processor_node` → `path_generator_node` → `coverage_visualizer_node` → `coverage_manager_node` |
| `open_coverage.launch.py` | **OpenNav polygon bridge** – converts safe map to polygon input | `map_processor_node` → `open_coverage_path` |
| `f2c.launch.py` | **F2C polygon pipeline** – converts safe map to WKT, then plans with Fields2Cover | `map_processor_node` → `open_coverage_path` → `f2c_path_gen_node` → `coverage_visualizer_node` |

### Modular pipeline data flow

```
/map (OccupancyGrid)
  │
  ▼
┌──────────────────────┐
│  map_processor_node  │  Erode free space by clearance_m
└──────────┬───────────┘
           │  /coverage/safe_map (OccupancyGrid)
           ▼
┌──────────────────────┐
│  path_generator_node │  Generate lawnmower waypoints
└──────────┬───────────┘
           │  /coverage/path (Path)
           ▼
┌──────────────────────────────┐
│  coverage_visualizer_node    │  Publish MarkerArray for RViz
└──────────────────────────────┘
           │  /coverage/waypoint_markers (MarkerArray)
           ▼
┌──────────────────────────────┐
│  coverage_manager_node       │  (optional) Send to Nav2
└──────────────────────────────┘
```

---

## Nodes

### `coverage_node` (monolithic)

> **File:** `coverage_planner/coverage_node.py`

Self-contained node that subscribes to `/map`, computes the coverage path, publishes it, and optionally executes it via Nav2. After processing the first map message, it unsubscribes and re-publishes the path + markers every 1 s (for late-joining RViz subscribers).

**Key behaviour:**
- Applies morphological erosion (elliptical kernel) to the free-space mask using `clearance_m`.
- Generates lawnmower waypoints with the `Segment` class.
- When `execute_coverage` is `true`, extracts **key poses** (corners where orientation changes) and navigates via `goToPose` → `goThroughPoses`.

---

### `map_processor_node`

> **File:** `coverage_planner/map_processor_node.py`

Subscribes to `/map`, inflates obstacles via morphological erosion, and publishes a binary safe/occupied grid on `/coverage/safe_map`. Re-publishes the latest safe map every 1 s.

| Subscribes | Publishes |
|---|---|
| `/map` (`OccupancyGrid`) | `/coverage/safe_map` (`OccupancyGrid`) |

---

### `path_generator_node`

> **File:** `coverage_planner/path_generator_node.py`

Subscribes to `/coverage/safe_map`, runs the lawnmower algorithm, and publishes a dense `Path` on `/coverage/path`. Re-publishes every 1 s.

| Subscribes | Publishes |
|---|---|
| `/coverage/safe_map` (`OccupancyGrid`) | `/coverage/path` (`Path`) |

---

### `coverage_visualizer_node`

> **File:** `coverage_planner/coverage_visualizer_node.py`

Subscribes to `/coverage/path` and publishes a `MarkerArray` containing:
- **SPHERE_LIST** – orange dots at every waypoint.
- **TEXT_VIEW_FACING** – numbered labels at every Nth waypoint (~50 labels max).

| Subscribes | Publishes |
|---|---|
| `/coverage/path` (`Path`) | `/coverage/waypoint_markers` (`MarkerArray`) |

---

### `coverage_manager_node`

> **File:** `coverage_planner/coverage_manager_node.py`

Subscribes to `/coverage/path`. When `execute_coverage` is `true`, it:
1. Waits for Nav2 to become active.
2. Extracts **sparse key poses** (corners where `orientation.z` changes) from the dense path.
3. Navigates to the first pose via `goToPose`.
4. Executes the remaining key poses via `goThroughPoses`.

| Subscribes | Publishes |
|---|---|
| `/coverage/path` (`Path`) | *(none – drives Nav2 via action client)* |

---

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | Input | SLAM-produced map |
| `/coverage/safe_map` | `nav_msgs/OccupancyGrid` | Internal | Obstacle-inflated safe free-space map |
| `/coverage/path` | `nav_msgs/Path` | Internal / Output | Dense lawnmower waypoints |
| `/coverage/waypoint_markers` | `visualization_msgs/MarkerArray` | Output | RViz debug markers |
| `/coverage/opennav_boundary` | `geometry_msgs/PolygonStamped` | Output | Largest safe-region polygon for OpenNav coverage |
| `/coverage/opennav_wkt` | `std_msgs/String` | Output | WKT polygon/multipolygon, including obstacle holes |
| `/coverage/opennav_polygons` | `visualization_msgs/MarkerArray` | Output | Green boundary and red hole markers |
| `/coverage/f2c_swaths` | `visualization_msgs/MarkerArray` | Output | F2C coverage swaths without transition connector lines |
| `/coverage_path` | `nav_msgs/Path` | Output (monolithic) | Path from `coverage_node` |
| `/coverage_points` | `visualization_msgs/MarkerArray` | Output (monolithic) | Markers from `coverage_node` |

All topics use **QoS: TRANSIENT_LOCAL / RELIABLE** so late-joining subscribers receive the last message.

---

## Parameters

### `coverage_node` (monolithic launch)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `map_topic` | string | `/map` | Map input topic |
| `path_topic` | string | `/coverage_path` | Path output topic |
| `marker_topic` | string | `/coverage_points` | Marker output topic |
| `clearance_m` | double | `0.35` | Obstacle inflation radius (meters) |
| `spacing_m` | double | `0.30` | Distance between sweep rows and between waypoints along a row |
| `min_segment_length_m` | double | `0.50` | Segments shorter than this are discarded |
| `execute_coverage` | bool | `false` | If `true`, sends path to Nav2 for execution |

### `map_processor_node`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `clearance_m` | double | `0.3` | Obstacle inflation radius (meters) |

### `path_generator_node`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `map_topic` | string | `/coverage/safe_map` | Safe map input topic |
| `path_topic` | string | `/coverage/path` | Path output topic |
| `spacing_m` | double | `0.3` | Row spacing and waypoint interval |
| `min_segment_length_m` | double | `0.2` | Free-space row intervals shorter than this are discarded |

### `coverage_manager_node`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `path_topic` | string | `/coverage/path` | Path input topic |
| `execute_coverage` | bool | `false` | If `true`, sends extracted key poses to Nav2 |

### Config file: `config/coverage.yaml`

```yaml
map_processor_node:
  ros__parameters:
    clearance_m: 0.3

path_generator_node:
  ros__parameters:
    spacing_m: 0.5
    min_segment_length_m: 0.2
```

---

## Launch Files

### `coverage.launch.py` – Monolithic

Starts only `coverage_node`. Best for quick testing.

```bash
ros2 launch coverage_planner coverage.launch.py
ros2 launch coverage_planner coverage.launch.py execute_coverage:=true clearance_m:=0.30 spacing_m:=0.25
```

| Argument | Default |
|---|---|
| `use_sim_time` | `false` |
| `execute_coverage` | `false` |
| `clearance_m` | `0.35` |
| `spacing_m` | `0.30` |

### `zigzag.launch.py` – Modular pipeline

Starts the 4-node pipeline. Parameters are loaded from `config/coverage.yaml`.

```bash
ros2 launch coverage_planner zigzag.launch.py
ros2 launch coverage_planner zigzag.launch.py execute_coverage:=true use_sim_time:=false
```

| Argument | Default |
|---|---|
| `use_sim_time` | `true` |
| `execute_coverage` | `false` |

### `open_coverage.launch.py` – OpenNav polygon bridge

Converts `/coverage/safe_map` into the polygon form needed by `opennav_coverage`.

```bash
ros2 launch coverage_planner open_coverage.launch.py
```

Outputs the largest safe connected region on `/coverage/opennav_boundary`, all safe regions as WKT on `/coverage/opennav_wkt`, and RViz markers on `/coverage/opennav_polygons`.

---

## Algorithm – How It Works

### 1. Map Processing (morphological erosion)

The raw occupancy grid marks cells as **free (0)**, **occupied (100)**, or **unknown (−1)**. The planner:

1. Extracts a binary mask: `free = (grid == 0)`.
2. Computes an erosion kernel radius: `r = clearance_m / resolution` pixels.
3. Erodes the free mask with an **elliptical structuring element** using OpenCV – this shrinks free space inward from walls so the robot centre never comes within `clearance_m` of an obstacle.

Coverage debug outputs such as `/coverage/safe_map`, `/coverage/path`, and markers use timestamp `0` so RViz uses the latest TF for static map-style visualization.

For the F2C launch, obstacle handling depends on `config/f2c.yaml`: increase `clearance_m` for more wall margin, lower `contour_simplification_px` to preserve wall detail, and lower `min_hole_area_m2` if small obstacles are ignored. The F2C node also validates generated path samples against `/coverage/safe_map` before publishing.

F2C can optimize the sweep angle from the extracted polygon by setting `use_best_swath_angle: true`. If coverage misses strips, reduce `coverage_width_m`; if RViz looks too dense, increase `path_step_m`; if turns clip obstacles, increase `clearance_m` or `min_turning_radius_m`.

`f2c.launch.py` runs `open_coverage_path` first, then `f2c_path_gen_node` consumes `/coverage/opennav_wkt`. WKT can contain multiple disconnected safe regions. Set `input_mode: map` in `config/f2c.yaml` only if you want F2C to rebuild polygons directly from `/coverage/safe_map`.

For indoor maps, use `path_output_mode: swaths` to publish straight swath passes and avoid large Dubins oval turns. Use `path_output_mode: dubins` only when continuous curved turns are desired and enough free space exists for them.

For production-style navigation, keep `repair_unsafe_connections: false` and let Nav2 plan between swaths/cells. Visualize `/coverage/f2c_swaths` as `MarkerArray` to see clean coverage lines without RViz path connector diagonals.

### 2. Boustrophedon decomposition and path generation

The modular `path_generator_node` first decomposes the safe grid into cells, then generates lawnmower sweeps per cell. A cell boundary is created when scanline connectivity changes: for example, when one free interval splits into two around an obstacle, or two intervals merge again after the obstacle.

The core algorithm uses:

```
Segment(y, x1, x2)   ← a horizontal span from pixel x1 to x2 on row y
Cell                 ← a group of connected segments between connectivity changes
```

**Steps:**

1. **Scan every row** – find contiguous free-space intervals.
2. **Track overlap with the previous row** – if exactly one previous interval overlaps one current interval, the same cell continues.
3. **Split/merge at critical rows** – if one interval becomes many, many become one, or topology changes, new cells start.
4. **Order cells** – visit cells greedily by nearest segment.
5. **Sweep each cell** – sample rows every `spacing_m`, then alternate left-to-right and right-to-left passes.
6. **Connect sweeps** – add L-shaped transition waypoints between rows and between cells.

**Waypoint format:** `(x_px, y_px, yaw)` – yaw is `0` for left-to-right, `π` for right-to-left, `±π/2` for vertical transitions.

MVP limitation: transition waypoints are generated geometrically. They should be validated against `/coverage/safe_map` before real robot execution.

### 3. Coordinate conversion

Pixel coordinates are converted to map frame using map origin, resolution, and origin yaw.

### 4. Nav2 execution (optional)

When `execute_coverage` is enabled:

1. Extract **key poses** – only keep waypoints where the orientation changes (corners/turns), reducing hundreds of dense waypoints to ~tens of sparse goals.
2. Navigate to the **first pose** via `goToPose`.
3. Execute remaining key poses via `goThroughPoses` (uses Nav2's behavior tree with recovery actions).

---

## How to Run

### Prerequisites

- ROS 2 (Humble or later)
- Nav2 stack (`nav2_simple_commander`)
- A SLAM-produced map being published on `/map`
- Python dependencies: `numpy`, `opencv-python`

### Build

```bash
cd ~/SensQ/ros2_ws
colcon build --packages-select coverage_planner
source install/setup.bash
```

### Visualize only (no robot movement)

```bash
# Monolithic
ros2 launch coverage_planner coverage.launch.py

# OR modular pipeline
ros2 launch coverage_planner zigzag.launch.py
```

Then in RViz add:
- **Path** display → topic: `/coverage_path` or `/coverage/path`
- **MarkerArray** display → topic: `/coverage_points` or `/coverage/waypoint_markers`

### Execute on robot

```bash
# Make sure Nav2 is running first, then:
ros2 launch coverage_planner zigzag.launch.py execute_coverage:=true use_sim_time:=false
```

---

## Improvement Ideas

### High Priority

| Area | Problem | Suggested Improvement |
|---|---|---|
| **Start from robot pose** | Path always starts at the first segment (top-left of the map), causing a long initial transit | Start from the segment nearest the robot's current pose (subscribe to `/amcl_pose` or use `tf`) |
| **No progress tracking** | If the robot is interrupted mid-coverage, the entire mission must restart | Track which segments/waypoints have been completed; persist state so coverage can resume |
| **Busy-wait during execution** | `while not self.navigator.isTaskComplete(): pass` blocks the executor thread | Use `async` feedback callbacks or a timer-based polling loop to avoid CPU spin-lock |
| **No obstacle re-checking** | Path is generated once from the initial map; new obstacles that appear during execution are ignored | Re-subscribe to map updates and re-plan uncovered regions dynamically |

### Medium Priority

| Area | Problem | Suggested Improvement |
|---|---|---|
| **Coverage efficiency** | Simple lawnmower produces many overlapping transitions in non-rectangular rooms | Implement **boustrophedon cellular decomposition** – split the map into trapezoidal cells and plan optimal sweeps per cell |
| **No rotation optimization** | Sweep direction is always horizontal; L-shaped rooms may be more efficiently covered at an angle | Compute the map's principal axis (PCA / minimum bounding rectangle) and rotate the sweep accordingly |
| **Transition path quality** | Jumps between disconnected regions use Euclidean distance heuristic and straight-line waypoints | Use Nav2's `ComputePathToPose` to plan collision-free transitions between disconnected zones |
| **Duplicate code** | `coverage_node.py` duplicates the full algorithm from the modular nodes | Refactor shared logic (lawnmower, Segment, pixel_to_map) into a shared utility module |
| **pixel_to_map inconsistency** | The modular `path_generator_node` ignores origin rotation; the monolithic `coverage_node` handles it | Unify to always handle origin rotation |

### Low Priority / Nice-to-Have

| Area | Suggested Improvement |
|---|---|
| **Coverage percentage metric** | Publish a `Float32` with the percentage of free space covered by the path |
| **Service interface** | Add `/start_coverage`, `/pause_coverage`, `/cancel_coverage` services (noted in original README) |
| **Database node** | Implement the planned `coverage_database_node` to log mission history (map metadata, path, result) |
| **Configurable sweep direction** | Allow setting sweep axis (horizontal / vertical / custom angle) via parameter |
| **Multi-room ordering** | Use a TSP solver to optimize the order in which disconnected regions are visited |
| **Real-time visualization** | Color waypoints green/red as they are reached/skipped during execution |
