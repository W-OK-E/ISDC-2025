import numpy as np
from scipy.ndimage import gaussian_filter

def compute_potential_field(depth_grid, goal, obstacle_weight=1.0, goal_weight=1.0):
    # Repulsive field: penalizes shallow depth
    obstacle_field = gaussian_filter(1 / (depth_grid + 1e-6), sigma=5)

    # Attractive field: encourages movement toward the goal
    goal_field = np.zeros_like(depth_grid)
    goal_field[goal] = 1
    goal_field = gaussian_filter(goal_field, sigma=10)

    # Combine fields with weights
    potential_field = obstacle_weight * obstacle_field - goal_weight * goal_field
    return potential_field


from queue import PriorityQueue

def a_star_with_potential(depth_grid, start, goal):
    rows, cols = depth_grid.shape
    potential_field = compute_potential_field(depth_grid, goal)

    # Movement directions: 8-connected grid
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    # A* initialization
    open_list = PriorityQueue()
    open_list.put((0, start))  # (priority, (x, y))
    came_from = {}
    cost_so_far = {start: 0}

    while not open_list.empty():
        _, current = open_list.get()
        if current == goal:
            break  # Goal reached

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            # Ensure the neighbor is within bounds
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue

            # Compute new cost
            # depth_cost = 1 / (depth_grid[neighbor] + 1e-6)  # Base traversal cost
            depth_cost = depth_grid[neighbor] 
            potential_cost = potential_field[neighbor]      # Potential field cost
            new_cost = cost_so_far[current] + depth_cost + potential_cost

            # Check if this path to neighbor is better
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)  # A* heuristic
                open_list.put((priority, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal)

def heuristic(node, goal):
    # Euclidean distance heuristic
    return np.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

def reconstruct_path(came_from, start, goal):
    # Backtrack from goal to start to reconstruct the path
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


import matplotlib.pyplot as plt
import os
def visualize_path(depth_grid, path,f_name = 'path.png',save = False):
    if(os.path.exists(f_name)):
        depth_grid = cv2.imread(f_name)
        depth_grid = cv2.cvtColor(depth_grid,cv2.COLOR_BGR2GRAY)
        depth_grid = depth_grid /(depth_grid.max()+2)
        plt.imshow(depth_grid, cmap='viridis')
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, color='red')  # Path overlay
        # plt.title("Path on Depth Grid")
        # plt.colorbar(label="Depth")
        plt.axis('off')
        plt.savefig(f_name)
    else:
        path_x, path_y = zip(*path)
        plt.imshow(depth_grid, cmap='viridis')
        plt.plot(path_y, path_x, color='red')  # Path overlay
        # plt.title("Path on Depth Grid")
        # plt.colorbar(label="Depth")
        plt.axis('off')
        plt.savefig(f_name)

if __name__ == "__main__":
    import cv2
    import yaml
    config = None
    with open('/home/summer/Desktop/van$/stitch.yaml','r') as file:
        config = yaml.safe_load(file)
    for file in config:   
        depth_grid = cv2.imread(file)
        depth_grid = cv2.cvtColor(depth_grid,cv2.COLOR_BGR2GRAY)
        depth_grid = depth_grid /(depth_grid.max()+2)
        saved_path = depth_grid
        if(len(config[file]) <= 1):
            print("Only one marker detected No Path planning involved")
            break
        path = []
        for idx in range(len(config[file])):
            print("Processing index:",idx)
            start = config[file][idx]
            goal =  config[file][idx+1]
            start = tuple(map(int,start.split(',')))
            goal = tuple(map(int,goal.split(',')))
            path.extend(a_star_with_potential(depth_grid, start, goal))
            if(idx+2 == len(config[file])):
                break

        visualize_path(saved_path, path,save=True)
            
