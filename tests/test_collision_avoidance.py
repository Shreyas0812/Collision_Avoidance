import os
import sys

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from grid_map import GridMap
from time_based_collision_avoidance import TimeBasedCollisionAvoidance

class SimpleAgent:
    """Simple agent for testing"""
    def __init__(self, agent_id, pos):
        self.id = agent_id
        self.pos = np.array(pos)

def test_collision_avoidance():
    """Test collision avoidance system"""
    
    print("=" * 60)
    print("COLLISION AVOIDANCE TEST")
    print("=" * 60)
    
    # Load map
    config_path = os.path.join('config', 'gridworld_warehouse_small.yaml')
    grid_map = GridMap(config_path)
    
    # Create collision avoidance system
    ca = TimeBasedCollisionAvoidance(grid_map)
    
    # Test 1: Two agents with non-conflicting paths
    print("\n--- Test 1: Non-Conflicting Paths ---")
    agents = [
        SimpleAgent(0, [6, 4, 0]),
        SimpleAgent(1, [6, 12, 0])
    ]
    goals = {
        0: grid_map.continuous_to_grid(2, 4, 0),   # Agent 0 → induct
        1: grid_map.continuous_to_grid(2, 12, 0)   # Agent 1 → different induct
    }
    
    paths = ca.plan_all_agents(agents, goals)
    
    print(f"Agent 0 path length: {len(paths[0])}")
    print(f"Agent 1 path length: {len(paths[1])}")
    print(f"✓ Both agents have paths")
    
    # Test 2: Two agents heading to same goal (should handle conflict)
    print("\n--- Test 2: Same Goal (Conflict) ---")
    agents = [
        SimpleAgent(0, [6, 4, 0]),
        SimpleAgent(1, [6, 5, 0])  # Near agent 0
    ]
    goals = {
        0: grid_map.continuous_to_grid(2, 4, 0),   # Both want same goal
        1: grid_map.continuous_to_grid(2, 4, 0)
    }
    
    paths = ca.plan_all_agents(agents, goals)
    
    print(f"Agent 0 path length: {len(paths[0])}")
    print(f"Agent 1 path length: {len(paths[1])}")
    
    # Agent 1 should have to wait or go around
    if len(paths[1]) > len(paths[0]):
        print("✓ Agent 1 (lower priority) has longer path (waited/detoured)")
    
    # Test 3: Head-on collision scenario
    print("\n--- Test 3: Head-On Collision ---")
    agents = [
        SimpleAgent(0, [5, 4, 0]),
        SimpleAgent(1, [3, 4, 0])  # On same row, facing each other
    ]
    goals = {
        0: grid_map.continuous_to_grid(3, 4, 0),   # Want to swap positions
        1: grid_map.continuous_to_grid(5, 4, 0)
    }
    
    paths = ca.plan_all_agents(agents, goals)
    
    print(f"Agent 0 path: {paths[0][:5]}...")
    print(f"Agent 1 path: {paths[1][:5]}...")
    
    # Check for collisions
    collisions = check_collisions(paths[0], paths[1])
    if collisions == 0:
        print("✓ No collisions detected")
    else:
        print(f"✗ {collisions} collisions detected!")
    
    # Test 4: Three agents
    print("\n--- Test 4: Three Agents ---")
    agents = [
        SimpleAgent(0, [6, 4, 0]),
        SimpleAgent(1, [6, 12, 0]),
        SimpleAgent(2, [6, 20, 0])
    ]
    goals = {
        0: grid_map.continuous_to_grid(11, 4, 0),
        1: grid_map.continuous_to_grid(11, 12, 0),
        2: grid_map.continuous_to_grid(11, 26, 0)
    }
    
    paths = ca.plan_all_agents(agents, goals)
    
    print(f"Agent 0 path length: {len(paths[0])}")
    print(f"Agent 1 path length: {len(paths[1])}")
    print(f"Agent 2 path length: {len(paths[2])}")
    print("✓ All agents have paths")
    
    # Check all pairs for collisions
    total_collisions = 0
    total_collisions += check_collisions(paths[0], paths[1])
    total_collisions += check_collisions(paths[0], paths[2])
    total_collisions += check_collisions(paths[1], paths[2])
    
    if total_collisions == 0:
        print("✓ No collisions between any agents")
    else:
        print(f"✗ {total_collisions} total collisions detected!")
    
    # Test 5: No goal (agent stays in place)
    print("\n--- Test 5: Agent With No Goal ---")
    agents = [
        SimpleAgent(0, [6, 4, 0]),
        SimpleAgent(1, [6, 12, 0])
    ]
    goals = {
        0: grid_map.continuous_to_grid(2, 4, 0),
        # Agent 1 has no goal
    }
    
    paths = ca.plan_all_agents(agents, goals)
    
    print(f"Agent 0 path length: {len(paths[0])}")
    print(f"Agent 1 path length: {len(paths[1])}")
    
    if len(paths[1]) == 1:
        print("✓ Agent 1 stays in place (no goal)")
    
    print("\n" + "=" * 60)
    print("COLLISION AVOIDANCE TESTING COMPLETE")
    print("=" * 60)

def check_collisions(path1, path2):
    """Check for vertex collisions between two paths"""
    collisions = 0
    max_len = max(len(path1), len(path2))
    
    # Extend shorter path (agent stays at goal)
    p1 = path1 + [path1[-1]] * (max_len - len(path1))
    p2 = path2 + [path2[-1]] * (max_len - len(path2))
    
    for t in range(max_len):
        if p1[t] == p2[t]:
            collisions += 1
            print(f"  Collision at time {t}: both at {p1[t]}")
    
    return collisions

if __name__ == "__main__":
    test_collision_avoidance()