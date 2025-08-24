import numpy as np
import random
import math

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

def euclidean_distance(n1, n2):
    return math.hypot(n1.x - n2.x, n1.y - n2.y)

def is_free(x, y, occ_grid, resolution, origin, threshold=20):
    mx = int((x - origin.position.x) / resolution)
    my = int((y - origin.position.y) / resolution)
    if 0 <= mx < occ_grid.shape[1] and 0 <= my < occ_grid.shape[0]:
        return occ_grid[my, mx] < threshold
    return False

def rrt(start, goal, occ_grid, resolution, origin, max_iter=100, step_size=0.65, goal_radius=0.7):
    tree = [Node(start[0], start[1])]
    for _ in range(max_iter):
        rand_x = random.uniform(origin.position.x, origin.position.x + occ_grid.shape[1] * resolution)
        rand_y = random.uniform(origin.position.y, origin.position.y + occ_grid.shape[0] * resolution)
        rand_node = Node(rand_x, rand_y)

        # Find nearest node in the tree
        nearest = min(tree, key=lambda node: euclidean_distance(node, rand_node))

        # Steer towards random node
        angle = math.atan2(rand_node.y - nearest.y, rand_node.x - nearest.x)
        new_x = nearest.x + step_size * math.cos(angle)
        new_y = nearest.y + step_size * math.sin(angle)
        new_node = Node(new_x, new_y, parent=nearest)

        if not is_free(new_x, new_y, occ_grid, resolution, origin):
            continue

        tree.append(new_node)

        if euclidean_distance(new_node, Node(goal[0], goal[1])) < goal_radius:
            # Path found
            path = [(new_node.x, new_node.y)]
            while new_node.parent:
                new_node = new_node.parent
                path.append((new_node.x, new_node.y))
            path.reverse()
            return path
    return None  # No path found
