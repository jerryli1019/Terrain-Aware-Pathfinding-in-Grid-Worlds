from __future__ import print_function
from heapq import *

ACTIONS = [(0,1),(1,0),(0,-1),(-1,0)]

class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.grid.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = [(0, self.grid.start)]
            self.frontier_node = {self.grid.start:0}
            self.explored = {}
        elif self.type == "astar":
            init_cost = abs(self.grid.start[0]-self.grid.goal[0]) + abs(self.grid.start[1]-self.grid.goal[1])
            self.frontier = [(init_cost, self.grid.start)]
            self.frontier_node = {self.grid.start:init_cost}
            self.explored = {}

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = True 
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop()

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored.append(current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.explored and n not in self.frontier:
                    self.previous[n] = current
                    if n == self.grid.goal:
                        self.finished = True
                        return
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True

    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current = self.frontier.pop(0)

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored.append(current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle and n not in self.explored and n not in self.frontier:
                    self.previous[n] = current
                    if n == self.grid.goal:
                        self.finished = True
                        return
                    self.frontier.append(n)
                    self.grid.nodes[n].color_frontier = True

    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current_data = heappop(self.frontier)
        current, cost = current_data[1], current_data[0]

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored[current] = (cost, current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                curr_cost = self.grid.nodes[n].cost() + cost
                if not self.grid.nodes[n].puddle and n not in self.explored and n not in self.frontier_node:
                    self.previous[n] = current
                    heappush(self.frontier, (curr_cost, n))
                    self.frontier_node[n] = curr_cost
                    self.grid.nodes[n].color_frontier = True
                elif not self.grid.nodes[n].puddle and n in self.frontier_node and curr_cost < self.frontier_node[n]:
                    self.previous[n] = current
                    heappush(self.frontier, (curr_cost, n))
                    self.frontier_node[n] = curr_cost
    
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        current_data = heappop(self.frontier)
        current, r_cost = current_data[1], current_data[0]
        h_cost = abs(current[0]-self.grid.goal[0]) + abs(current[1]-self.grid.goal[1])

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False
        self.explored[current] = (r_cost, current)

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                curr_h_cost = abs(n[0]-self.grid.goal[0]) + abs(n[1]-self.grid.goal[1])
                curr_cost = self.grid.nodes[n].cost() + r_cost - h_cost + curr_h_cost
                if not self.grid.nodes[n].puddle and n not in self.explored and n not in self.frontier_node:
                    self.previous[n] = current
                    heappush(self.frontier, (curr_cost, n))
                    self.frontier_node[n] = curr_cost
                    self.grid.nodes[n].color_frontier = True
                elif not self.grid.nodes[n].puddle and n in self.frontier_node and curr_cost < self.frontier_node[n]:
                    self.previous[n] = current
                    heappush(self.frontier, (curr_cost, n))
                    self.frontier_node[n] = curr_cost
