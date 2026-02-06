import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from grid_map import GridMap
from astar import AStar

def test_astar():
    """Test A* pathfinding"""
    
    print("=" * 60)
    print("A* PATHFINDING TEST")
    print("=" * 60)
    
    # Load map
    config_path = os.path.join('config', 'gridworld_warehouse_small.yaml')
    grid_map = GridMap(config_path)
    
    # Create A* planner
    astar = AStar(grid_map)
    
    # Test 1: Agent to nearby induct station
    print("\n--- Test 1: Agent to Induct Station ---")
    start = grid_map.continuous_to_grid(6, 4, 0)  # Agent 0 position
    goal = grid_map.continuous_to_grid(2, 4, 0)   # Induct station 1
    
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Straight-line distance: {astar.heuristic(start, goal)} cells")
    
    path = astar.find_path(start, goal)
    
    if path:
        print(f"✓ Path found: {len(path)} steps")
        print(f"  First 5 steps: {path[:5]}")
        print(f"  Last 5 steps: {path[-5:]}")
    else:
        print("✗ No path found!")
    
    # Test 2: Induct to eject station
    print("\n--- Test 2: Induct to Eject Station ---")
    start = grid_map.continuous_to_grid(2, 4, 0)   # Induct station
    goal = grid_map.continuous_to_grid(11, 4, 0)   # Eject station
    
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Straight-line distance: {astar.heuristic(start, goal)} cells")
    
    path = astar.find_path(start, goal)
    
    if path:
        print(f"✓ Path found: {len(path)} steps")
        print(f"  First 5 steps: {path[:5]}")
        print(f"  Last 5 steps: {path[-5:]}")
    else:
        print("✗ No path found!")
    
    # Test 3: Longer path across warehouse
    print("\n--- Test 3: Cross-Warehouse Path ---")
    start = grid_map.continuous_to_grid(6, 4, 0)    # Agent position
    goal = grid_map.continuous_to_grid(19, 26, 0)   # Far eject station
    
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Straight-line distance: {astar.heuristic(start, goal)} cells")
    
    path = astar.find_path(start, goal)
    
    if path:
        print(f"✓ Path found: {len(path)} steps")
        print(f"  Path length vs straight-line: {len(path)-1} vs {astar.heuristic(start, goal)}")
    else:
        print("✗ No path found!")
    
    # Test 4: Invalid goal (obstacle)
    print("\n--- Test 4: Path to Obstacle (should fail) ---")
    start = grid_map.continuous_to_grid(6, 4, 0)
    goal = (0, 0, 0)  # Border obstacle
    
    print(f"Start: {start}")
    print(f"Goal: {goal} (obstacle)")
    
    path = astar.find_path(start, goal)
    
    if path is None:
        print("✓ Correctly rejected path to obstacle")
    else:
        print("✗ Should not find path to obstacle!")
    
    # Test 5: Between two adjacent eject stations (should route through free space)
    print("\n--- Test 5: Between Adjacent Eject Stations ---")
    start = grid_map.continuous_to_grid(11, 4, 0)   # Eject station
    goal = grid_map.continuous_to_grid(12, 4, 0)    # Adjacent eject station
    
    print(f"Start: {start} (eject)")
    print(f"Goal: {goal} (eject)")
    print(f"Direct distance: {astar.heuristic(start, goal)} cell")
    
    path = astar.find_path(start, goal)
    
    if path:
        print(f"✓ Path found: {len(path)} steps")
        if len(path) > 2:
            print(f"  ✓ Path routes through free space (not direct)")
            print(f"  Path: {path}")
        else:
            print(f"  ✗ Path is direct (should go through free space)")
    else:
        print("✗ No path found!")
    
    print("\n" + "=" * 60)
    print("A* TESTING COMPLETE")
    print("=" * 60)

if __name__ == "__main__":
    test_astar()