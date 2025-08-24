import numpy as np
from nav_msgs.msg import OccupancyGrid

def detect_frontiers(occupancy_grid: OccupancyGrid):
    """
    Detect frontier points (boundaries between known free space and unknown space).
    Returns a list of (x, y) in map frame.
    """
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    resolution = occupancy_grid.info.resolution
    origin = occupancy_grid.info.origin

    data = np.array(occupancy_grid.data).reshape((height, width))
    frontiers = []

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if data[y, x] == 0:  # free space
                neighbors = data[y-1:y+2, x-1:x+2].flatten()
                if -1 in neighbors:  # unknown around free space = frontier
                    map_x = origin.position.x + (x * resolution)
                    map_y = origin.position.y + (y * resolution)
                    frontiers.append((map_x, map_y))

    return frontiers
