#!/usr/bin/env python3
# import heapq
# import numpy as np

# class AStarPlanner:
#     def __init__(self, grid, resolution):
#         self.grid = grid
#         self.resolution = resolution

#     def heuristic(self, a, b):
#         return abs(a[0] - b[0]) + abs(a[1] - b[1])

#     def get_neighbors(self, node):
#         neighbors = [
#             (node[0] + dx, node[1] + dy)
#             for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
#         ]
#         return [
#             neighbor for neighbor in neighbors
#             if 0 <= neighbor[0] < self.grid.shape[0]
#             and 0 <= neighbor[1] < self.grid.shape[1]
#             and self.grid[neighbor[0], neighbor[1]] == 0
#         ]

#     def plan(self, start, goal):
#         open_set = []
#         heapq.heappush(open_set, (0, start))
#         came_from = {}
#         g_score = {start: 0}
#         f_score = {start: self.heuristic(start, goal)}

#         while open_set:
#             _, current = heapq.heappop(open_set)

#             if current == goal:
#                 path = []
#                 while current in came_from:
#                     path.append(current)
#                     current = came_from[current]
#                 path.append(start)
#                 return path[::-1]

#             for neighbor in self.get_neighbors(current):
#                 tentative_g_score = g_score[current] + 1
#                 if tentative_g_score < g_score.get(neighbor, float('inf')):
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g_score
#                     f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
#                     heapq.heappush(open_set, (f_score[neighbor], neighbor))

#         return None
# scripts/a_star_planner.py
import heapq

class Node:
    def __init__(self, x, y, cost, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def a_star(start, goal, grid):
    open_list = []
    closed_list = set()
    start_node = Node(start[0], start[1], 0)
    goal_node = Node(goal[0], goal[1], 0)
    heapq.heappush(open_list, (0, start_node))

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if (current_node.x, current_node.y) in closed_list:
            continue

        closed_list.add((current_node.x, current_node.y))

        if current_node.x == goal_node.x and current_node.y == goal_node.y:
            return reconstruct_path(current_node)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor_x = current_node.x + dx
            neighbor_y = current_node.y + dy
            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]) and grid[neighbor_x][neighbor_y] == 0:
                neighbor = Node(neighbor_x, neighbor_y, current_node.cost + 1, current_node)
                heapq.heappush(open_list, (neighbor.cost + heuristic(neighbor, goal_node), neighbor))

    return None

def heuristic(node, goal_node):
    return abs(node.x - goal_node.x) + abs(node.y - goal_node.y)

def reconstruct_path(node):
    path = []
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]

if __name__ == "__main__":
    grid = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start = (0, 0)
    goal = (4, 4)
    path = a_star(start, goal, grid)
    print("Path:", path)