import heapq
from collections import defaultdict

class TimeBasedCollisionAvoidance:
    """
    TimeBasedCollisionAvoidance for collision free MAPF
    """

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.reservations = {} # {(x, y, z, t): agent_id}

    def clear_reservations(self):
        """
        Clears all reservations.
        """
        self.reservations = {}
    
    def heuristic(self, pos1, pos2):
        """
        Computes the Manhattan distance heuristic between two positions.
        
        :param pos1: (x1, y1, z1)
        :param pos2: (x2, y2, z2)
        :return: Manhattan distance
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) + abs(pos1[2] - pos2[2])
    
    def is_reserved(self, x, y, z, t, agent_id):
        """
        Checks if a cell is reserved at a given time for a different agent.
        
        :param x: x coordinate
        :param y: y coordinate
        :param z: z coordinate
        :param t: time step
        :param agent_id: ID of the agent checking the reservation
        :return: True if reserved by another agent, False otherwise
        """
        key = (x, y, z, t)
        if key in self.reservations:
            return self.reservations[key] != agent_id
        return False
    
    def has_edge_conflict(self, curr_pos, next_pos, t, agent_id):
        """
        Checks for edge conflicts (swapping positions) between two agents.
        
        :param curr_pos: Current position of the agent (x1, y1, z1)
        :param next_pos: Next position of the agent (x2, y2, z2)
        :param t: current time step
        :param agent_id: ID of the agent checking for conflicts
        :return: True if there is an edge conflict, False otherwise
        """
        if self.is_reserved(*next_pos, t-1, agent_id) and self.is_reserved(*curr_pos, t, agent_id):
            # Another agent is moving from next_pos to curr_pos at the same time
            # Basically a swap conflict
            return True
        return False
    
    def reserve_path(self, path, agent_id):
        """
        Reserves the path for the agent.
        
        :param path: List of positions [(x1, y1, z1), (x2, y2, z2), ...]
        :param agent_id: ID of the agent
        """
        for t, pos in enumerate(path):
            key = (*pos, t)
            self.reservations[key] = agent_id

        # Reserve goal position for all future timesteps 
        if path:
            goal_pos = path[-1]
            for future_t in range(len(path), len(path) + 1000): # Arbitrary large number to reserve goal
                key = (*goal_pos, future_t)
                self.reservations[key] = agent_id
    
    def plan_path_with_reservations(self, start, goal, agent_id, max_time=1000):
        """
        Path plan with A* search considering time-based reservations.

        :param start: Starting position (x, y, z)
        :param goal: Goal position (x, y, z)
        :param agent_id: ID of the agent
        :param max_time: Maximum time steps to search
        :return: List of positions [(x1, y1, z1), (x2, y2, z2), ...] or None if no path found
        """

        if start == goal:
            return [start]
        
        if not self.grid_map.is_valid_cell(*start) or not self.grid_map.is_valid_cell(*goal):
            return None
        
        # Priority queue for A* search: (f_score, counter, (position, time_step), path)
        counter = 0
        open_set = [(0, counter, (start, 0), [start])]
        closed_set = set()