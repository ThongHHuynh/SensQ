# Headland and transition tuning

This records the planner/controller changes from the simulation run that reached
coverage segment 48 of 198.

## Changes

### Smooth headlands

- Added `headland_overlap_m: 0.05`.
- Shortened both adjoining straight rows by the overlap distance.
- Reconstructed those 5 cm sections as tangent lead-in and lead-out portions of
  the turn.
- Used a cubic curve between the two tangent points.
- Retained collision checking and square-turn fallback.
- Increased `min_turning_radius_m` from `0.12` to `0.16`.
- Reduced `path_spacing_m` to `0.025` for denser curve samples.

### Trackable transitions

- Split every inter-cell transit from the preceding coverage path.
- Split transit direction changes of 20 degrees or more into separate
  `FollowPath` goals.
- Kept headland curves continuous.

This prevents Nav2 path pruning from jumping between overlapping transit legs
and prevents pure pursuit from cutting across sharp A* corners.

### Controller used during that validation run

- Kept DWB for navigation to the initial coverage entry.
- Switched coverage tracking to Regulated Pure Pursuit:
  - desired speed: `0.20 m/s`
  - lookahead: `0.12 m`
  - minimum regulated speed: `0.05 m/s`
  - collision horizon: `0.60 s`
- Disabled `use_rotate_to_heading`. The robot follows planned forward curves
  instead of oscillating around the in-place alignment threshold.
- Used pose-aware progress checks with `0.02 m` translation or `0.10 rad`
  rotation and an `8 s` allowance.
- Used `0.15 m` coverage endpoint tolerance.

The current configuration has since been reverted to the original DWB-only
controller, progress checker, goal checker, costmap, and velocity-smoother
settings. The planner geometry changes above remain.

### Costmap

- Kept robot radius at `0.10 m`.
- During that run, local/global inflation radius was `0.25 m` with scaling
  factor `5.0`.
- Did not reduce collision protection to bypass blocked paths.

## Validation result

- Plan: 11,343 poses, 198 segments, 16 cells, and `262.464 m`.
- Estimated safe-area coverage: `95.96%`.
- Turns: 85 curves, 3 square fallbacks, and 18 inter-cell transits.
- The robot passed the previously failing headlands and first transit.
- The run reached segment 48 of 198.
- All 17 package tests passed.

Segment 48 revealed an environment mismatch: the top row was free in the
static map but lethal in Nav2's live global costmap. A live-only `0.30 m`
safety-margin experiment was canceled and never saved. The generalized planner
configuration remains:

```yaml
robot_radius_m: 0.10
safety_margin_m: 0.05
headland_overlap_m: 0.05
min_turning_radius_m: 0.16
```

## Remaining work

- Validate planned segments against current Nav2 costmap data.
- Replan invalid remaining coverage without map-specific margins or
  coordinates.
- Rebuild and run the complete test suite after controller changes.
- Complete an isolated full-map run without retries or errors.
