import math
import numpy as np
import heapq
import matplotlib.pyplot as plt
import matplotlib.colors as colors  
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




    def generate_path(self,cell_grid, x_new, y_new):
        path = []

        while cell_grid[x_new][y_new].parent_x != self.thymio_pos[0] or cell_grid[x_new][y_new].parent_y != self.thymio_pos[1] :

              path.append((x_new,y_new))
              x_new, y_new = cell_grid[x_new][y_new].parent_x, cell_grid[x_new][y_new].parent_y 

        path.append((x_new, y_new))
        path.reverse()


        image_chemin = np.copy(self.map)
        for x, y in path :
            image_chemin[x][y] = 2;

        return path, image_chemin


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
        new_path, image_chemin = self.find_path()

        image_chemin[self.thymio_pos[0]][self.thymio_pos[1]] = 3;
        image_chemin[self.goal_pos[0]][self.goal_pos[1]] = 4;
        return new_path, image_chemin





        
def main() :
    plt.ion(); 

    grid = [

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
    ]   
    

    pos = [0, 0]
    dest = [8, 0]
    grid[pos[0]][pos[1]] = 3;
    grid[dest[0]][dest[1]] = 4;

    GlobalNav = global_nav(grid, pos, dest)
    colorsList = ['black', 'white', 'green', 'blue', 'yellow']
    CustomCmap = colors.ListedColormap(colorsList)

    path, image_chemin = GlobalNav.find_path()
    image_chemin[pos[0]][pos[1]] = 3;
    image_chemin[dest[0]][dest[1]] = 4;

    image = plt.matshow(grid, cmap=CustomCmap, vmin=0, vmax=4);
    plt.pause(0.1);

    image.set_data(image_chemin);

    plt.pause(1);

    grid2[pos[0]][pos[1]] = 3;
    grid2[dest[0]][dest[1]] = 4;
    image.set_data(grid2);
    plt.pause(1);


    path2, image_chemin2 = GlobalNav.update_new_obstacle(grid2);
    image.set_data(image_chemin2);



    plt.ioff();
    plt.show();
    print(path)



main()


