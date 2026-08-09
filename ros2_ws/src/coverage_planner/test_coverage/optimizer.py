from __future__ import annotations

import math

from test_coverage.geometry import append_path
from test_coverage.models import Cell, CellPath, PathKind, PathPoint, Point2D
from test_coverage.transitions import TransitionPlanner


def optimize_route(
    cells: list[Cell],
    variants: dict[int, list[CellPath]],
    start: Point2D,
    transition_planner: TransitionPlanner,
    enable_2opt: bool,
) -> tuple[list[CellPath], float]:
    if not cells:
        return [], math.inf

    best_order = greedy_cell_order(cells, start)
    best_proxy_cost = centroid_route_cost(best_order, start)

    if enable_2opt and 3 < len(cells) <= 20:
        improved = True
        passes = 0
        while improved and passes < 3:
            improved = False
            passes += 1
            for first in range(0, len(best_order) - 2):
                for last in range(first + 1, len(best_order) - 1):
                    candidate_order = (
                        best_order[:first]
                        + list(reversed(best_order[first:last + 1]))
                        + best_order[last + 1:]
                    )
                    candidate_cost = centroid_route_cost(candidate_order, start)
                    if candidate_cost + 1e-6 < best_proxy_cost:
                        best_order = candidate_order
                        best_proxy_cost = candidate_cost
                        improved = True

    # Run the obstacle-aware endpoint dynamic program only for the selected
    # cell order. Re-running grid A* for every 2-opt permutation is needlessly
    # expensive and does not materially improve the coarse cell ordering.
    return select_variants(best_order, variants, start, transition_planner)


def greedy_cell_order(cells: list[Cell], start: Point2D) -> list[Cell]:
    remaining = list(cells)
    order = []
    current = start
    while remaining:
        selected = min(
            remaining,
            key=lambda cell: math.hypot(
                cell.centroid.x - current.x, cell.centroid.y - current.y
            ),
        )
        order.append(selected)
        remaining.remove(selected)
        current = selected.centroid
    return order


def centroid_route_cost(order: list[Cell], start: Point2D) -> float:
    cost = 0.0
    current = start
    for cell in order:
        cost += math.hypot(
            cell.centroid.x - current.x,
            cell.centroid.y - current.y,
        )
        current = cell.centroid
    return cost


def select_variants(
    order: list[Cell],
    variants: dict[int, list[CellPath]],
    start: Point2D,
    transition_planner: TransitionPlanner,
) -> tuple[list[CellPath], float]:
    if not order:
        return [], math.inf

    costs: dict[int, float] = {}
    parents: list[dict[int, int]] = []
    first_variants = variants[order[0].cell_id]
    for index, variant in enumerate(first_variants):
        entry_cost = transition_planner.distance(start, variant.points[0].point)
        costs[index] = entry_cost + variant.length_px
    parents.append({})

    for order_index in range(1, len(order)):
        previous_variants = variants[order[order_index - 1].cell_id]
        current_variants = variants[order[order_index].cell_id]
        next_costs: dict[int, float] = {}
        parent_for_stage: dict[int, int] = {}

        for current_index, current_variant in enumerate(current_variants):
            best_cost = math.inf
            best_parent = -1
            for previous_index, previous_variant in enumerate(previous_variants):
                transition_cost = transition_planner.distance(
                    previous_variant.points[-1].point,
                    current_variant.points[0].point,
                )
                candidate = (
                    costs.get(previous_index, math.inf)
                    + transition_cost
                    + current_variant.length_px
                )
                if candidate < best_cost:
                    best_cost = candidate
                    best_parent = previous_index
            next_costs[current_index] = best_cost
            parent_for_stage[current_index] = best_parent
        costs = next_costs
        parents.append(parent_for_stage)

    final_index = min(costs, key=costs.get)
    selected_indices = [final_index]
    for stage in range(len(order) - 1, 0, -1):
        selected_indices.append(parents[stage][selected_indices[-1]])
    selected_indices.reverse()

    selected = [
        variants[cell.cell_id][variant_index]
        for cell, variant_index in zip(order, selected_indices)
    ]
    return selected, costs[final_index]


def assemble_route(
    selected: list[CellPath],
    transition_planner: TransitionPlanner,
) -> list[PathPoint]:
    result: list[PathPoint] = []
    for cell_path in selected:
        if result:
            transition = transition_planner.connect(
                result[-1], cell_path.points[0], cell_path.cell_id
            )
            if not transition:
                return []
            transition = [
                PathPoint(
                    point.x,
                    point.y,
                    point.yaw,
                    PathKind.TRANSIT,
                    cell_path.cell_id,
                )
                for point in transition
            ]
            append_path(result, transition)
        append_path(result, cell_path.points)
    return result
