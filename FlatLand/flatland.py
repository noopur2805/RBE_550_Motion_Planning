

import matplotlib.pyplot as plt
import random
import numpy as np
%matplotlib inline

# Class to craete grid by avoiding obstacles

class Grid:
    def __init__(self, grid_dim, occupancy_percent=30, weighted=False):
        self.grid_dim = grid_dim
        self.occupancy_percent = occupancy_percent
        self.weighted = weighted
        
    # SHAPE "I"
    def shape1(self, matrix, xpos, ypos):
        matrix[xpos][ypos] = 1
        matrix[xpos+1][ypos] = 1
        matrix[xpos+2][ypos] = 1
        matrix[xpos+3][ypos] = 1
        return matrix


    # SHAPE "L"
    def shape2(self, matrix, xpos, ypos):
        matrix[xpos][ypos] = 1
        matrix[xpos][ypos+1] = 1
        matrix[xpos+1][ypos+1] = 1
        matrix[xpos+2][ypos+1] = 1
        return matrix

    # SHAPE "S"
    def shape3(self, matrix, xpos, ypos):
        matrix[xpos][ypos] = 1
        matrix[xpos+1][ypos] = 1
        matrix[xpos+1][ypos+1] = 1
        matrix[xpos+2][ypos+1] = 1
        return matrix

    # SHAPE "T"
    def shape4(self, matrix, xpos, ypos):
        matrix[xpos][ypos+1] = 1
        matrix[xpos+1][ypos] = 1
        matrix[xpos+1][ypos+1] = 1
        matrix[xpos+2][ypos+1] = 1
        return matrix

    def create_grid(self):
        if self.weighted == False:
            grid_matrix = [[0] * self.grid_dim for count in range(self.grid_dim)] 
        else:
            grid_matrix = (np.random.rand(self.grid_dim, self.grid_dim)*100+100).astype(int).tolist()
        #  Filling 10%, 50% and 70% of the grid
        # occupancy_percent = [30]#, 50, 70]

#         for op in occupancy_percent:
        num_shapes = int(self.grid_dim * self.grid_dim * (self.occupancy_percent / 100) / 4)
        draw_shapes = {1: self.shape1,
                        2: self.shape2,
                        3: self.shape3,
                        4: self.shape4}

        for i in range(num_shapes):
            random_num = random.randint(1,4)
            xpos = random.randint(0, self.grid_dim-4)
            ypos = random.randint(0, self.grid_dim-2)

            grid_matrix = draw_shapes[random_num](grid_matrix, xpos, ypos)
            # for i in range(grid_dim):
            #     print(grid_matrix[i])
            # print()    

        return grid_matrix
    

GRID_DIM = 128

# Testing for occupancy percentages 25%, 50% and 75%
grid = Grid(GRID_DIM, occupancy_percent=25, weighted=False)
grid_matrix = grid.create_grid()
plt.imshow(grid_matrix, cmap="Blues")
plt.show()

%matplotlib qt

# =================================================================

# Class to craete graph and execute all algorithms like BFS, DFS, Djikstra and random planner

from queue import Queue
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import math
%matplotlib qt

class Graph:
    def __init__(self, num_nodes, weighted=False):
        self.num_nodes = num_nodes
        self.nodes = list(range(self.num_nodes))
        self.adj_list = {node: set() for node in self.nodes}
        self.paths_explored = []
        self.weighted = weighted
        
    def create_edge(self, parent_node, child_node, weight=1):
        if self.weighted == False:
            self.adj_list[parent_node].add(child_node)
        else:
            self.adj_list[parent_node].add((child_node, weight))
        
    def create_graph(self, grid_matrix, grid_dim):
        for i in range(self.num_nodes):
            if (grid_matrix[i//grid_dim][i%grid_dim] == 1):
                continue

            if ((i-1 >= 0) and (i%grid_dim != 0) 
                        and (grid_matrix[(i-1)//grid_dim][(i-1)%grid_dim] != 1)):   # Left
                graph.create_edge(i, i-1)
            if ((i+1 < grid_dim*grid_dim) and ((i+1)%grid_dim != 0) 
                        and (grid_matrix[(i+1)//grid_dim][(i+1)%grid_dim] != 1)):   # Right
                graph.create_edge(i, i+1)
            if ((i-grid_dim >= 0) 
                        and (grid_matrix[(i-grid_dim)//grid_dim][(i-grid_dim)%grid_dim] != 1)):  # Up
                graph.create_edge(i, i-grid_dim)
            if ((i+grid_dim < grid_dim*grid_dim) 
                        and (grid_matrix[(i+grid_dim)//grid_dim][(i+grid_dim)%grid_dim] != 1)):  # Down
                graph.create_edge(i, i+grid_dim)

        
    def print_adj_list(self):
        for key in self.adj_list.keys():
            print("Node", key, ": ", self.adj_list[key])
        
    def construct_path(self, parent_node, goal_node):
        path = []
        path.append(goal_node)
        while parent_node[goal_node] is not None:
            path.append(parent_node[goal_node]) 
            goal_node = parent_node[goal_node]
        path.reverse()
        return path 

    def bfs(self, start_node, goal_node):

        visited = set()
        queue = []
        
        queue.append(start_node)
        visited.add(start_node)
        
        parent_node = {}
        parent_node[start_node] = None

        reached_goal = False
        path = []
        
        while (len(queue) > 0):
            curr_node = queue.pop(0)
            if (curr_node == goal_node):
                reached_goal = True
                break
            for child_node in self.adj_list[curr_node]:
                if child_node not in visited:
                    queue.append(child_node)
                    parent_node[child_node] = curr_node
                    visited.add(child_node)

            path = self.construct_path(parent_node, child_node)
            self.paths_explored.append(path)
        
        
        print("Total iterations consumed: ", len(visited))
        print("Total path cost: ", len(self.paths_explored[-1]))
        if (reached_goal):
            print("Goal FOUND!!")
            return path
        else:
            print("Goal point UNREACHABLE!!")
            
    def dfs(self, start_node, goal_node):
        visited = set()
        stack = []

        stack.append(start_node)
        visited.add(start_node)

        parent_node = {}
        parent_node[start_node] = None

        reached_goal = False
        path = []
        
        while (len(stack) > 0):
            curr_node = stack.pop(-1)
            if (curr_node == goal_node):
                reached_goal = True
                break
            for child_node in self.adj_list[curr_node]:
                if child_node not in visited:
                    stack.append(child_node)
                    parent_node[child_node] = curr_node
                    visited.add(child_node)
                    
                path = self.construct_path(parent_node, child_node)
                self.paths_explored.append(path)
            
        print("Total iterations consumed: ", len(visited))
        print("Total path cost: ", len(self.paths_explored[-1]))
        if (reached_goal):
            print("Goal FOUND!!")
            return path
        else:
            print("Goal point UNREACHABLE!!")
            
    def djikstra(self, start_node, goal_node):

        visited = set()
        priority_queue = []
        
        priority_queue.append(start_node)
        visited.add(start_node)
        
        parent_node = {}
        parent_node[start_node] = None

        reached_goal = False
        path = []
            
        while (len(priority_queue) > 0):
            curr_node = priority_queue.pop(0)
            if (curr_node == goal_node):
                reached_goal = True
                break
            
            if (len(self.adj_list[curr_node]) > 0):
                for child_node, wt in sorted(self.adj_list[curr_node], key=lambda tup: tup[1]): # Sorting cost by ascending order and choosing the child node with minimal cost amongst all
                    if child_node not in visited:
                        priority_queue.append(child_node)
                        parent_node[child_node] = curr_node
                        visited.add(child_node)

                    path = self.construct_path(parent_node, child_node)
                    self.paths_explored.append(path)
                    
        print("Total iterations consumed: ", len(visited))
        print("Total nodes: ", len(self.paths_explored[-1]))
        minimum_cost = sum([grid_matrix[n//GRID_DIM][n%GRID_DIM] for n in self.paths_explored[-1][1:]])
        # print('Minimum cost of path (taking random costs on vacant nodes): ', minimum_cost)

        if (reached_goal):
            print("Goal FOUND!!")
            return path
        else:
            print("Goal point UNREACHABLE!!")
            
    # Not in use
    def euclidean_distance(self, start_node, goal_node):
        start_node_x = start_node // GRID_DIM
        start_node_y = start_node % GRID_DIM
        goal_node_x = goal_node // GRID_DIM
        goal_node_y = goal_node % GRID_DIM
        
        dist = math.sqrt(((start_node_x - goal_node_x) * (start_node_x - goal_node_x)) 
                    + ((start_node_y - goal_node_y) * (start_node_y - goal_node_y)))
        return dist
        
    # Obselete
    def random_path_generator(self, start_node, goal_node):
        
        visited = set()
        queue = []
        
        queue.append(start_node)
        visited.add(start_node)
        
        parent_node = {}
        parent_node[start_node] = None

        reached_goal = False
        path = []
        
        while (len(queue) > 0):
            curr_node = queue.pop(0)
            if (curr_node == goal_node):
                reached_goal = True
                break
            distances = dict([(child_node, self.euclidean_distance(child_node, goal_node)) \
                                             for child_node in self.adj_list[curr_node] \
                                             if child_node not in visited])
            for child_node in [k for k, v in sorted(distances.items(), key=lambda item: item[1])]:
                if child_node not in visited:
                    queue.append(child_node)
                    parent_node[child_node] = curr_node
                    visited.add(child_node)

                path = self.construct_path(parent_node, child_node)
                self.paths_explored.append(path)

        return path

    # Final random planner
    def random_path_last(self, start_node, goal_node):
        global grid_matrix
        
        max_iterations = 100
        
        obstacles_indices = np.where(np.array(grid_matrix) == 1)
        obstacle_nodes = set(obstacles_indices[0]*GRID_DIM + obstacles_indices[1])
        visited = set()
        path = []
        path.append(start_node)
        visited.add(start_node)
        
        parent_node = {}
        parent_node[start_node] = None

        reached_goal = False
        curr_node = start_node
        i = 0
        while (not reached_goal or i < max_iterations): # if >0, that means some child node is still left to be visited
            if (curr_node == goal_node):
                reached_goal = True
                break
            next_node = random.choice(list(self.adj_list[curr_node]))
            if next_node in obstacle_nodes:
                continue
            path.append(next_node)
            parent_node[next_node] = curr_node
            curr_node = next_node
            
            self.paths_explored.append(path)
            i += 1
        
        print("Total iterations consumed: ", len(visited))
        print("Total path cost: ", len(self.paths_explored[-1]))
        if (reached_goal):
            print("Goal FOUND!!")
        else:
            print("Goal point UNREACHABLE!!")
            
        return path

    
# Find indices of thse nodes where obtacles are not present and assign them as START and END nodes
obstacles_indices = np.where(np.array(grid_matrix) == 1)
obstacle_nodes = set(obstacles_indices[0]*GRID_DIM + obstacles_indices[1])

START_NODE = random.choice(list(set(range(GRID_DIM*GRID_DIM)) - obstacle_nodes))
lower_quartile = [START_NODE - 500 if START_NODE - 500 > 0 else 0][0]
upper_quartile = [START_NODE + 500 if START_NODE + 500 < GRID_DIM*GRID_DIM else (GRID_DIM * GRID_DIM) -1][0]

left_nodes = list(set(range(GRID_DIM*GRID_DIM)) - obstacle_nodes.union(set([START_NODE])))
END_NODE = random.choice(list(filter(lambda x: lower_quartile < x < upper_quartile, left_nodes)))
print("Start node:", START_NODE, " End node: ", END_NODE)

# lower_quartile, upper_quartile

# =============================================================================


# Create graph from the grid
total_nodes = GRID_DIM * GRID_DIM
graph = Graph(total_nodes, weighted=False)
graph.create_graph(grid_matrix, GRID_DIM)

# Calling graph algorithms
from matplotlib.animation import FuncAnimation
resultant_path = graph.bfs(START_NODE, END_NODE)



# =========================================================================================


# Animation of grid searching algorithms

from matplotlib.animation import FuncAnimation, PillowWriter, ImageMagickWriter
from copy import deepcopy
import matplotlib.pyplot as plt
import numpy as np
%matplotlib qt

# plt.rcParams["figure.figsize"] = [10, 10]
plt.rcParams["figure.autolayout"] = True
fig, ax = plt.subplots()

def update(self):
    mat = deepcopy(grid_matrix)
    mat = [list(map(lambda x: x*100, m)) for m in mat]
    mat[START_NODE // GRID_DIM][START_NODE % GRID_DIM] = 50 # start node
    mat[END_NODE // GRID_DIM][END_NODE % GRID_DIM] = 80 # end node

    for path in graph.paths_explored:
        #print(path)
        for i in path[1:-1]:
            mat[i//GRID_DIM][i%GRID_DIM] = 25 # path nodes
        ax.imshow(mat, cmap = 'turbo')
        ax.set_axis_off()
        plt.pause(0.0001)

anim = FuncAnimation(fig, update, frames=50, repeat=False)
plt.show()
# anim.save('bfs_10.gif', writer='imagemagick')
# anim.save("test2.png", writer=ImageMagickWriter(fps=5, extra_args=['-loop', '1']))









