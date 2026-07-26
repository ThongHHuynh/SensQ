import numpy as np 
from coverage_planner_archive.coverage_geometry import Cell, Segment

def row_intervals(row, min_len_px):
    free_x = np.where(row == 0)[0]
    #if no free pixels, return empty list
    if len(free_x) == 0:
        return []
    # find difference between consecutive free pixels
    # if the difference is greater than 1, we have found a run
    # split the free pixels into runs
    runs = np.split(free_x, np.where(np.diff(free_x)>1)[0] + 1)

    return [ 
        (int(run[0]), int (run[-1]))
        for run in runs if len(run) >= min_len_px
    ]

"""Check if two intervals overlap."""
def overlaps(a,b):
    return a[0] <= b[1] and b[0] <= a[1]

"""Boustrophedon Decomposition"""
def decompose_cells(grid, min_len_px, min_cell_area_px):
    cells = {}
    previous = []
    next_cell_id = 0 

    # Check row intervals in each row
    for y in range(grid.shape[0]):
        current = row_intervals(grid[y], min_len_px)
        # -- if line is blocked -> previous = [] and continue to next line
        if not current:
            previous = []
            continue

        active = []

        for prev_indices, curr_indices in overlap_components(previous, current):
            if not curr_indices:
                continue

            if len(prev_indices) == 1 and len(curr_indices) == 1:
                cell_id = previous[prev_indices[0]].cell_id
                curr_idx = curr_indices[0]
                active.append(add_segment(cells, cell_id, y, current[curr_idx]))
                continue

            for curr_idx in curr_indices:
                cell_id = next_cell_id
                next_cell_id += 1
                active.append(add_segment(cells, cell_id, y, current[curr_idx]))
        previous = active
    
    #Filter out cells that are too small
    return [cell for cell in cells.values()
            if cell.area_px >= min_cell_area_px]

def overlap_components(previous, current):

    nodes = [("p", i) for i in range(len(previous))]
    nodes.extend([("c", i) for i in range(len(current))])
    #create dictionary to track which nodes connected to which
    edges = {node: [] for node in nodes}

    for prev_idx, prev in enumerate(previous):
        for curr_idx, curr in enumerate(current):
            if overlaps((prev.x1, prev.x2), curr):
                p_node = ("p", prev_idx)
                c_node = ("c", curr_idx)
                edges[p_node].append(c_node)
                edges[c_node].append(p_node)

    components = []
    visited = set()
    for node in nodes:
        if node in visited: 
            continue

        stack = [node]
        visited.add(node)
        prev_indices = []
        curr_indices = []

        while stack: 
            kind, idx  = stack.pop()
            if kind == "p":
                prev_indices.append(idx)
            else:
                curr_indices.append(idx)
            
            for neighbor in edges[(kind, idx)]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)

        components.append((prev_indices, curr_indices))
    return components
        
def add_segment(cells, cell_id, y, interval):
    cells.setdefault(cell_id, Cell(id=cell_id))
    # add a new segment to the cell
    segment = Segment(y, interval[0], interval[1], cell_id)
    cells[cell_id].segments.append(segment)
    return segment






        
