import math

from coverage_planner_archive.coverage_geometry import Waypoint


def remove_duplicate_waypoints(waypoints):
    cleaned = []
    for waypoint in waypoints:
        if not cleaned or cleaned[-1].x != waypoint.x or cleaned[-1].y != waypoint.y:
            cleaned.append(waypoint)
    return cleaned


def simplify_path_rdp(waypoints, epsilon_px):
    """Ramer-Douglas-Peucker simplification.

    Reduces waypoint count from A* output while preserving path shape.
    Keeps first and last waypoint. Keeps points where the perpendicular
    distance to the line segment exceeds epsilon_px.
    """
    if len(waypoints) <= 2:
        return list(waypoints)

    max_dist = 0.0
    max_idx = 0

    for i in range(1, len(waypoints) - 1):
        dist = perpendicular_distance(waypoints[i], waypoints[0], waypoints[-1])
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > epsilon_px:
        left = simplify_path_rdp(waypoints[:max_idx + 1], epsilon_px)
        right = simplify_path_rdp(waypoints[max_idx:], epsilon_px)
        return left[:-1] + right
    else:
        return [waypoints[0], waypoints[-1]]


def perpendicular_distance(point, line_start, line_end):
    dx = line_end.x - line_start.x
    dy = line_end.y - line_start.y
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        return math.hypot(point.x - line_start.x, point.y - line_start.y)

    t = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / length_sq
    t = max(0.0, min(1.0, t))

    proj_x = line_start.x + t * dx
    proj_y = line_start.y + t * dy
    return math.hypot(point.x - proj_x, point.y - proj_y)