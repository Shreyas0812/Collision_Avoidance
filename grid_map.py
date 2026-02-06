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
        self.depth = int(params['grid_depth'])
        self.resolution = float(params['grid_resolution'])

        # Create occupancy grid (0 = free, 1 = occupied)
        self.grid = np.zeros((self.depth, self.height, self.width), dtype=np.uint8)

        # Mark obstacles in the grid
        obstacles_regions_flat = params['obstacle_regions']
        self.mark_obstacles(obstacles_regions_flat)

        total_cells = self.width * self.height * self.depth
        obstacle_cells = np.sum(self.grid)
        print("Grid Initialized:")
        print(f" - Dimensions: {self.width} x {self.height} x {self.depth}")
        print(f" - Total Cells: {total_cells}")
        print(f" - Obstacle Cells: {obstacle_cells}")
        print(f" - Resolution: {self.resolution}")


    def mark_obstacles(self, obstacles_regions_flat):
        """
        Marks obstacle regions in the occupancy grid.
        
        :param obstacles_regions_flat: [start_x, start_y, start_z, end_x, end_y, end_z, ...]
        """
        num_regions = len(obstacles_regions_flat) // 6
        for i in range(num_regions):
            x_min = int(obstacles_regions_flat[i * 6 + 0] / self.resolution)
            y_min = int(obstacles_regions_flat[i * 6 + 1] / self.resolution)
            z_min = int(obstacles_regions_flat[i * 6 + 2] / self.resolution)
            x_max = int(obstacles_regions_flat[i * 6 + 3] / self.resolution)
            y_max = int(obstacles_regions_flat[i * 6 + 4] / self.resolution)
            z_max = int(obstacles_regions_flat[i * 6 + 5] / self.resolution)

            for z in range(z_min, z_max + 1):
                for y in range(y_min, y_max + 1):
                    for x in range(x_min, x_max + 1):
                        if self._in_bounds(x, y, z):
                            self.grid[z, y, x] = 1  # Mark as occupied
        

    def _in_bounds(self, x, y, z):
        return 0 <= x < self.width and 0 <= y < self.height and 0 <= z < self.depth
            
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_dir, 'config', 'gridworld_warehouse_small.yaml')
    grid_map = GridMap(config_path)
    print("Grid map created with dimensions:", grid_map.width, "x", grid_map.height, "x", grid_map.depth)