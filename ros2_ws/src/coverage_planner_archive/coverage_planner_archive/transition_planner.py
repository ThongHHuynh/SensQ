import heapq
import math

from coverage_planner_archive.coverage_geometry import Waypoint
from coverage_planner_archive.grid_utils import is_safe

# 8-connected neighbor offsets with movement costs.
_NEIGHBORS_8 = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
]
_NEIGHBORS_4 = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
]


def connect(current, target, grid, validation_step_px, sample_px,
            use_diagonal=True, max_astar_iterations=50_000):
    """Connect two waypoints safely.

    Tries cheap L-shaped routes first. Falls back to A* grid search
    only when L-shaped routes are blocked by obstacles.
    """
    for route in l_shaped_routes(current, target):
        if route_safe(route, grid, validation_step_px):
            return route[1:]

    return astar_connect(current, target, grid, sample_px, use_diagonal,
                         max_iterations=max_astar_iterations)


def l_shaped_routes(current, target):
    return [
        [
            current,
            Waypoint(current.x, target.y, math.pi / 2.0 if target.y > current.y else -math.pi / 2.0),
            Waypoint(target.x, target.y, 0.0 if target.x > current.x else math.pi),
        ],
        [
            current,
            Waypoint(target.x, current.y, 0.0 if target.x > current.x else math.pi),
            Waypoint(target.x, target.y, math.pi / 2.0 if target.y > current.y else -math.pi / 2.0),
        ],
    ]


def route_safe(route, grid, step_px):
    return all(line_safe(route[i], route[i + 1], grid, step_px) for i in range(len(route) - 1))


def line_safe(a, b, grid, step_px):
    dist = max(abs(b.x - a.x), abs(b.y - a.y))
    steps = max(1, int(math.ceil(dist / max(1, step_px))))

    for i in range(steps + 1):
        t = i / steps
        x = int(round(a.x + (b.x - a.x) * t))
        y = int(round(a.y + (b.y - a.y) * t))
        if not is_safe(grid, x, y):
            return False

    return True


def astar_connect(current, target, grid, sample_px, use_diagonal=True,
                  max_iterations=50_000):
    """A* grid search with 8-connected neighbors.

    8-connected movement produces smoother diagonal paths instead of
    staircasing around obstacles. Falls back to 4-connected if disabled.

    Args:
        max_iterations: Hard cap on nodes expanded. Prevents unbounded
            search on large maps. Returns empty path when exceeded.
    """
    start = (int(round(current.x)), int(round(current.y)))
    goal = (int(round(target.x)), int(round(target.y)))
    if not is_safe(grid, *start) or not is_safe(grid, *goal):
        return []

    neighbor_offsets = _NEIGHBORS_8 if use_diagonal else _NEIGHBORS_4

    open_set = [(0.0, start)]
    came_from = {}
    cost = {start: 0.0}
    visited = set()
    iterations = 0

    while open_set:
        if iterations >= max_iterations:
            return []
        iterations += 1

        _, node = heapq.heappop(open_set)
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            route = reconstruct(came_from, node)
            return route_to_waypoints(route, target.yaw, sample_px)

        x, y = node
        for dx, dy, move_cost in neighbor_offsets:
            nx, ny = x + dx, y + dy
            if not is_safe(grid, nx, ny):
                continue
            if dx != 0 and dy != 0:
                if not is_safe(grid, x + dx, y) or not is_safe(grid, x, y + dy):
                    continue
            neighbor = (nx, ny)
            next_cost = cost[node] + move_cost
            if next_cost >= cost.get(neighbor, math.inf):
                continue
            came_from[neighbor] = node
            cost[neighbor] = next_cost
            priority = next_cost + octile(neighbor, goal)
            heapq.heappush(open_set, (priority, neighbor))

    return []


def reconstruct(came_from, node):
    route = [node]
    while node in came_from:
        node = came_from[node]
        route.append(node)
    route.reverse()
    return route


def route_to_waypoints(route, final_yaw, sample_px):
    waypoints = []
    last_direction = None

    for i in range(1, len(route)):
        prev = route[i - 1]
        node = route[i]
        direction = (node[0] - prev[0], node[1] - prev[1])
        is_turn = last_direction is not None and direction != last_direction
        is_sample = i % max(1, sample_px) == 0
        is_goal = i == len(route) - 1

        if is_turn or is_sample or is_goal:
            waypoints.append(Waypoint(node[0], node[1], direction_to_yaw(direction)))

        last_direction = direction

    if waypoints:
        waypoints[-1].yaw = final_yaw
    return waypoints


def direction_to_yaw(direction):
    dx, dy = direction
    return math.atan2(dy, dx)


def octile(a, b):
    """Octile distance heuristic for 8-connected grids.

    Admissible and consistent for 8-connected A* with cardinal cost 1
    and diagonal cost sqrt(2).
    """
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + 0.414 * min(dx, dy)
