import math
import numpy as np

def occupancy_grid_to_array(msg):
    return np.array(msg.data, dtype=np.int16).reshape(
        msg.info.height,
        msg.info.width,
    )

def meters_to_pixels(value_m, resolution, minimum=1):
    return max(minimum, int(value_m / resolution))

#return true if the pixel is within the grid and is free (0), false otherwise
def is_safe(grid, x, y):
    return 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1] and grid[y, x] == 0

def pixel_to_map(x_px, y_px, msg):
    resolution = msg.info.resolution
    origin = msg.info.origin

    yaw = quaternion_to_yaw(origin.orientation)
    local_x = (x_px + 0.5) * resolution
    local_y = (y_px + 0.5) * resolution

    x = origin.position.x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
    y = origin.position.y + local_x * math.sin(yaw) + local_y * math.cos(yaw)
    return x, y

def map_to_pixel(x_map, y_map, msg):
    """Convert map-frame coordinates to pixel coordinates."""
    resolution = msg.info.resolution
    origin = msg.info.origin
    yaw = quaternion_to_yaw(origin.orientation)

    dx = x_map - origin.position.x
    dy = y_map - origin.position.y

    local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)

    px = int(local_x / resolution)
    py = int(local_y / resolution)

    return px, py

def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)

def transpose_grid(grid):
    """Transpose a 2D numpy array."""
    return np.ascontiguousarray(grid.T)