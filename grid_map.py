import numpy as np
import yaml
import os

class GridMap:
    """
    Warehouse config to grid-based occupancy map
    """

    def __init__(self, config_path):
        with open(config_path) as file:
            config = yaml.safe_load(file)

        # Access the parameters
        params = config['create_gridworld_node']['ros__parameters']

        self.width = int(params['grid_width'])
        self.height = int(params['grid_height'])
        self.resolution = params['grid_resolution']

        # Create occupancy grid (0 = free, 1 = occupied)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

        # Mark obstacles in the grid
        obstacles_regions_flat = params['obstacle_regions']
        
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, 'config', 'gridworld_warehouse_small.yaml')
    grid_map = GridMap(config_path)
    print("Grid map created with dimensions:", grid_map.width, "x", grid_map.height)