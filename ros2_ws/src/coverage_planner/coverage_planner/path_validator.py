from coverage_planner.transition_planner import line_safe


def unsafe_edge_count(waypoints, grid, validation_step_px):
    unsafe = 0
    for i in range(len(waypoints) - 1):
        if not line_safe(waypoints[i], waypoints[i + 1], grid, validation_step_px):
            unsafe += 1
    return unsafe