from __future__ import annotations
import warnings
from dataclasses import dataclass, field


@dataclass(frozen=True)
class GridPoint:
    x: int
    y: int

@dataclass
class Segment:
    y: int
    x1: int
    x2: int
    cell_id: int

@dataclass
class Cell:
    id: int
    segments: list[Segment] = field(default_factory=list)

    @property
    def area_px(self) -> int:
        return sum(segment.x2 - segment.x1 + 1 for segment in self.segments)
    @property
    def centroid(self) -> tuple[float, float]:
        if not self.segments:
            warnings.warn(f"Cell {self.id} has no segments; centroid defaults to (0, 0).")
            return (0.0, 0.0)
        total_x = sum((s.x1 + s.x2) / 2.0 for s in self.segments)
        total_y = sum(float(s.y) for s in self.segments)
        n = len(self.segments)
        return (total_x / n, total_y / n)

@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float