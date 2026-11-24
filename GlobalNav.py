import math
import numpy as np
import heapq

class cells :
    def __init__(self):
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0
        self.parent_x = 0
        self.parent_y = 0




class global_nav :

    def __init__(map, thymio_pos, goal_pos):
        self.map = map #map of obstacles, binary
        self.thymio_pos = thymio_pos
        self.goal_pos = goal_pos
        self.direction = [(1, 0), (-1, 0), (0, 1), (0, -1)]


    def update_new_obstacles(map);
        self.map = map
        find_path()


    def find_path():
        columns = len(self.map[0])
        rows = len(self.map)

        # bouger dans self ? #################################################################################
        visited_cells = np.zeros(rows, columns)
        cell_grid = [[cells() for i in range (columns)] for j in range(rows)]

        x_start, y_start = self.thymio_pos

        cell_grid[x_start][y_start].f = 0
        cell_grid[x_start][y_start].g = 0
        cell_grid[x_start][y_start].h = 0
        cell_grid[x_start][y_start].parent_x = x_start
        cell_grid[x_start][y_start].parent_y = y_start

        cell_to_visit = []
        heapq.heappush(cell_to_visit, (0.0, x_start, y_start))

        while open_list :

            _, x_actu, y_actu = heapq.heappop(cell_to_visit)

            if visited_cells[x_actu][y_actu] == True : 
                continue

            visited_cells[x_actu][y_actu] = True

            for dx, dy in directions :
                x_new = x_actu + dx
                y_new = y_actu + dy

            if x_new == self.goal_pos[0] and y_new == self.goal_pos[1]
                cell_grid[x_new][y_new].parent_x = x_actu
                cell_grid[x_new][y_new].parent_y = y_actu
                generate_path(cell_grid, x_new, y_new)
                return path

            g_new = cell_grid[x_new][y_new].g + 1.0
            h_new = abs(x_new - thymio_pos[0]) + abs(y_new - thymio_pos[1])
            f_new = g_new + h_new

            if cell_grid[x_new][y_new].f > f_new
                cell_grid[x_new][y_new].f = f_new
                cell_grid[x_new][y_new].g = g_new
                cell_grid[x_new][y_new].h = h_new

                cell_grid[x_new][y_new].parent_x = x_actu
                cell_grid[x_new][y_new].parent_y = y_actu


                heapq.heappush(cell_to_visit, (f_new, x_new, y_new))




    #functions for a-star algorithm

    def generate_path(cell_grid, x_start, y_start):
        path = []

        while cell_grid







        
        


    def animation():




