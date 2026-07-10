import math

from coverage_planner.coverage_geometry import Waypoint

def order_cells(cells, start_x, start_y, use_2opt=True):
    """Order cells starting from the robot's actual position.
    Use greedy nearest-neighbor, refine with 2-opt to 
    reduce inter-cell travel distance."""

    ordered = []
    remaining = [cell for cell in cells if cell.segments]  # Filter out cells with no segments
    current = Waypoint(start_x, start_y, 0.0)
    while remaining: 
        cell = min(remaining, key=lambda c: distance_to_cell(c, current))
        ordered.append(cell)
        remaining.remove(cell)
        first = min(cell.segments, key=lambda s: (s.y, s.x1))
        current = Waypoint(first.x1, first.y, 0.0)

    if use_2opt and len(ordered) > 3:
        ordered = improve_order_2opt(ordered, start_x, start_y)

    return ordered

def improve_order_2opt(cells, start_x, start_y):
    """Local 2-opt improvement on cell visit order.

    Repeatedly reverses sub-sequences to reduce total travel.
    O(n^2) per pass but cell count is typically < 50.
    """
    def total_cost(order):
        cost = 0.0
        cx, cy = start_x, start_y
        for cell in order:
            cost += distance_to_cell(cell, Waypoint(cx, cy, 0.0))
            last = max(cell.segments, key=lambda s: s.y)
            cx, cy = last.x2, last.y
        return cost

    best = list(cells)
    best_cost = total_cost(best)
    improved = True

    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                candidate = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                candidate_cost = total_cost(candidate)
                if candidate_cost < best_cost:
                    best = candidate
                    best_cost = candidate_cost
                    improved = True

    return best

def distance_to_cell(cell, point):
    """
    Compute the squared distance from a point to the closest point on the cell's segments.
    """
    return min(
        min(
            (segment.x1 - point.x) ** 2 + (segment.y - point.y) **2,
            (segment.x2 - point.x) ** 2 + (segment.y - point.y) ** 2,
        )
        for segment in cell.segments
    )

def generate_cell_sweeps(cell, spacing_px, waypoint_spacing_px):
    """Generate boustrophedon sweep waypoints for a single cell.

    Waypoints within a sweep row are on the same free-space row
    and do not need transition planning.
    """
    sampled = sample_segments(cell.segments, spacing_px)
    waypoints = []
    direction = 1

    for segment in sampled:
        if direction == 1:
            start = Waypoint(segment.x1, segment.y, 0.0)
            end = Waypoint(segment.x2, segment.y, 0.0)
        else:
            start = Waypoint(segment.x2, segment.y, math.pi)
            end = Waypoint(segment.x1, segment.y, math.pi)

        row_wps = segment_waypoints(start, end, waypoint_spacing_px)

        if waypoints:
            waypoints.append(("transition", start))
        
        for wp in row_wps:
            waypoints.append(("sweep", wp))

        direction *= -1
    return waypoints



def sample_segments(segments, spacing_px):
    """sort and filter each horizontal segment"""
    #sorted by y, then x1
    ordered = sorted(segments, key=lambda s: (s.y, s.x1))
    sampled = []
    last_y = None

    #only keep segments that are at least spacing_px apart in y
    for segment in ordered:
        if last_y is None or segment.y - last_y >= spacing_px:
            sampled.append(segment)
            last_y = segment.y

    # check if we need to add the last segment
    if ordered and sampled[-1].y != ordered[-1].y:
        sampled.append(ordered[-1])
    return sampled

def segment_waypoints(start, end, step_px):
    if start.x <= end.x:
        xs = range(start.x, end.x, step_px)
    else:
        xs = range(start.x, end.x, -step_px)
    waypoints = [Waypoint(x, start.y, start.yaw) for x in xs]
    waypoints.append(end)  # Ensure the end point is included
    return waypoints
