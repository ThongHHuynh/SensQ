import numpy as np


class CoverageTracker:
    """Tracks which grid cells have been physically covered by the robot."""

    def __init__(self, grid, coverage_radius_px):
        self.free_mask = (grid == 0)
        self.covered = np.zeros_like(grid, dtype=bool)
        self.coverage_radius_px = coverage_radius_px
        self.total_free = int(np.sum(self.free_mask))

    def mark_covered(self, x_px, y_px):
        """Mark cells within coverage radius of robot position as covered."""
        r = int(self.coverage_radius_px)
        x_px = int(round(x_px))
        y_px = int(round(y_px))
        rows, cols = self.covered.shape

        y_min = max(0, y_px - r)
        y_max = min(rows, y_px + r + 1)
        x_min = max(0, x_px - r)
        x_max = min(cols, x_px + r + 1)

        yy, xx = np.ogrid[y_min:y_max, x_min:x_max]
        radius_mask = (xx - x_px) ** 2 + (yy - y_px) ** 2 <= r * r
        region = self.covered[y_min:y_max, x_min:x_max]
        region[radius_mask & self.free_mask[y_min:y_max, x_min:x_max]] = True

    @property
    def coverage_percentage(self):
        if self.total_free == 0:
            return 100.0
        return 100.0 * float(np.sum(self.covered & self.free_mask)) / self.total_free

    @property
    def covered_area_cells(self):
        return int(np.sum(self.covered & self.free_mask))

    def get_uncovered_regions(self):
        """Return mask of free cells that have not been covered."""
        return self.free_mask & ~self.covered
