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
    
    def find_path(self, start, goal):
        """
        Finds the shortest path from start to goal using A* algorithm.
        
        :param start: Tuple (x, y, z) for start grid coordinates
        :param goal: Tuple (x, y, z) for goal grid coordinates
        :return: List of grid coordinates representing the path from start to goal
        """
        if start == goal:
            return [start]

        if not self.grid_map.is_valid_cell(*start) or not self.grid_map.is_valid_cell(*goal):
            return []
        
        # Priority queue: (f_score, counter, current_pos, path)
        # Using counter to avoid comparison issues in heapq when f_scores are equal

        counter = 0
        open_set = [(0, counter, start, [start])]
        closed_set = set()

        while open_set:

            f_score, _, current_pos, path = heapq.heappop(open_set)

            # Already evaluated
            if current_pos in closed_set:
                continue
            
            # Reached goal
            if current_pos == goal:
                return path
            
            closed_set.add(current_pos)

            for neighbor in self.grid_map.get_neighbors(*current_pos):
                if neighbor in closed_set:
                    continue
                
                path_new = path + [neighbor]

                g_score = len(path_new) - 1 # Cost from start to neighbor
                h_score = self.heuristic(neighbor, goal)
                f_score = g_score + h_score
                
                counter += 1
                heapq.heappush(open_set, (f_score, counter, neighbor, path_new))

        return []  # No path found
    
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, 'config', 'gridworld_warehouse_small.yaml')
    
    grid_map = GridMap(config_path)
    astar = AStar(grid_map)
    
    start = (2, 4, 0)  # Starting grid coordinates
    goal = (11, 5, 0)  # Goal grid coordinates
    
    path = astar.find_path(start, goal)
    print("Path from start to goal:", path)