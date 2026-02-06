import heapq
import os 

from grid_map import GridMap

class AStar:
    """
    AStar path finding algorithm implementation for 3D grid maps.
    """
    def __init__(self, grid_map):
        self.grid_map = grid_map

    def heuristic(self, pos1, pos2):
        """
        Heuristic: Manhattan distance for grid-based pathfinding.
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) + abs(pos1[2] - pos2[2])
    
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, 'config', 'gridworld_warehouse_small.yaml')
    
    grid_map = GridMap(config_path)
    astar = AStar(grid_map)
    
    start = (0, 0, 0)  # Starting grid coordinates
    goal = (10, 10, 0)  # Goal grid coordinates
    
    # path = astar.find_path(start, goal)
    # print("Path from start to goal:", path)