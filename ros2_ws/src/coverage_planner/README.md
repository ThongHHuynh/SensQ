# Coverage Planner

A ROS 2 package that generates and (optionally) executes **lawnmower-style coverage paths** over a SLAM-produced occupancy grid.  
The robot sweeps every reachable free-space cell while respecting a configurable safety clearance from walls and obstacles.

---

## Table of Contents

- [Overview](#overview)
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
2. **Path Generation** – scan the safe grid row-by-row to build a zigzag (lawnmower) path.
3. **Visualization** – publish RViz markers so you can inspect waypoints before sending the robot.
4. **Execution** – (optional) send the path to Nav2 via `NavigateThroughPoses`.

---

## Architecture

The package ships **two architectures** selectable via launch file:

| Launch file | Architecture | Nodes started |
|---|---|---|
| `coverage.launch.py` | **Monolithic** – single node does everything | `coverage_node` |
| `zigzag.launch.py` | **Modular pipeline** – each stage is a separate node | `map_processor_node` → `path_generator_node` → `coverage_visualizer_node` → `coverage_manager_node` |

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

---

## Algorithm – How It Works

### 1. Map Processing (morphological erosion)

The raw occupancy grid marks cells as **free (0)**, **occupied (100)**, or **unknown (−1)**. The planner:

1. Extracts a binary mask: `free = (grid == 0)`.
2. Computes an erosion kernel radius: `r = clearance_m / resolution` pixels.
3. Erodes the free mask with an **elliptical structuring element** using OpenCV – this shrinks free space inward from walls so the robot centre never comes within `clearance_m` of an obstacle.

### 2. Lawnmower path generation

The core algorithm uses the `Segment` class to represent contiguous horizontal runs of free pixels at a given row:

```
Segment(y, x1, x2)   ← a horizontal span from pixel x1 to x2 on row y
```

**Steps:**

1. **Scan rows** – iterate through the safe free-space image every `spacing_px` rows. For each row, find contiguous runs of free pixels and create `Segment` objects. Discard segments shorter than `min_segment_length_px`.
2. **Start at the first segment** – sweep left-to-right, placing waypoints every `spacing_px` pixels.
3. **Move to the next connected row** – look one `spacing_px` row below (or above) for a segment that overlaps horizontally. If found, reverse the sweep direction (right-to-left) and add **L-shaped transition** waypoints (vertical move, then horizontal alignment) to avoid diagonal cuts.
4. **Reverse vertical direction** – if no connected segment is found below, try above.
5. **Jump to nearest unvisited segment** – if no connected segment exists in either direction, find the closest unvisited segment (by Euclidean distance) and start a new sweep from there.
6. **Repeat** until every segment is visited.

**Waypoint format:** `(x_px, y_px, yaw)` – yaw is `0` for left-to-right, `π` for right-to-left, `±π/2` for vertical transitions.

### 3. Coordinate conversion

Pixel coordinates are converted to map frame using:

```
x_map = origin.x + x_px * resolution
y_map = origin.y + y_px * resolution
```

The monolithic `coverage_node` additionally accounts for origin rotation via quaternion-to-yaw conversion.

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