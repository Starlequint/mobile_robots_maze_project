from importlib.resources import path
import math
import heapq
import numpy as np
import Affichage
import cv2

class cells :
    def __init__(self):
        self.f = float('inf')
        self.g = float('inf')
        self.h = 0
        self.parent_x = 0
        self.parent_y = 0




class global_nav :

    def __init__(self, map, thymio_pos, goal_pos):
        self.map = map #map of obstacles, binary
        self.thymio_pos = thymio_pos
        self.goal_pos = goal_pos
        self.directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]


    def path_taille_relle(self, path):
        scaled_path = [
        (75 + x * 150, 75 + y * 150)
        for x, y in path
        ]
        print(path[0])
        return scaled_path


    def generate_path(self,cell_grid, x_new, y_new):
        path = []

        while cell_grid[x_new][y_new].parent_x != self.thymio_pos[0] or cell_grid[x_new][y_new].parent_y != self.thymio_pos[1] :

              path.append((y_new,x_new))
              x_new, y_new = cell_grid[x_new][y_new].parent_x, cell_grid[x_new][y_new].parent_y 

        path.append((y_new, x_new))
        path.reverse()

        path = self.path_taille_relle(path)

        return path


    def find_path(self):
        columns = len(self.map[0])
        rows = len(self.map)
        visited_cells = np.zeros((rows, columns), dtype=bool)
        cell_grid = [[cells() for i in range (columns)] for j in range(rows)]

        x_start, y_start = self.thymio_pos

        cell_grid[x_start][y_start].f = 0
        cell_grid[x_start][y_start].g = 0
        cell_grid[x_start][y_start].h = 0
        cell_grid[x_start][y_start].parent_x = x_start
        cell_grid[x_start][y_start].parent_y = y_start

        cell_to_visit = []
        heapq.heappush(cell_to_visit, (0.0, x_start, y_start))

        while cell_to_visit :

            _, x_actu, y_actu = heapq.heappop(cell_to_visit)

            if visited_cells[x_actu][y_actu] == True : 
                continue

            visited_cells[x_actu][y_actu] = True

            for dx, dy in self.directions :
                x_new = x_actu + dx
                y_new = y_actu + dy	
                if not (0 <= x_new < rows and 0 <= y_new < columns):
                    continue
                if self.map[x_new][y_new] == 0:
                    continue

                if x_new == self.goal_pos[0] and y_new == self.goal_pos[1] :
                    cell_grid[x_new][y_new].parent_x = x_actu
                    cell_grid[x_new][y_new].parent_y = y_actu
                    path = self.generate_path(cell_grid, x_new, y_new)
                    return path

                g_new = cell_grid[x_actu][y_actu].g + 1.0
                h_new = abs(x_new - self.goal_pos[0]) + abs(y_new - self.goal_pos[1])
                f_new = g_new + h_new

                if cell_grid[x_new][y_new].f > f_new :
                    cell_grid[x_new][y_new].f = f_new
                    cell_grid[x_new][y_new].g = g_new
                    cell_grid[x_new][y_new].h = h_new

                    cell_grid[x_new][y_new].parent_x = x_actu
                    cell_grid[x_new][y_new].parent_y = y_actu


                    heapq.heappush(cell_to_visit, (f_new, x_new, y_new))


    def update_new_obstacle(self,map):
        self.map = map
        new_path = self.find_path()
        return new_path



def main() :

    grid = [
        [1, 1, 1, 1],
        [0, 0, 0, 1],
        [1, 1, 0, 1],
        [1, 1, 1, 1]
    ]


    '''grid = [

    [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0],

    [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1],

    [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
    
    [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0],

    [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1],
 
    [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1]
    ]   

    grid2 = [

    [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0],

    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],

    [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],

    [1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1],

    [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
    
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0],

    [0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
 
    [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
 
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1],
 
    [1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1]
    ]   '''
    

    pos = [0, 0]
    dest = [2, 1]
    GlobalNav = global_nav(grid, pos, dest)

    path = GlobalNav.find_path()
    print(path)
    image_with_path = Affichage.draw_path_on_image(cv2.imread("quadrillage.jpg"), path, (75, 75), 30, 20, np.array([[0, 0], [0, 0], [0, 0]], np.int32), grid)
    cv2.imshow("Image with Path", image_with_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


main()




