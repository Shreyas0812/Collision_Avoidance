import heapq
from collections import defaultdict

class TimeBasedCollisionAvoidance:
    """
    TimeBasedCollisionAvoidance for collision free MAPF
    """

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.reservations = {} # {(x, y, z, t): agent_id}