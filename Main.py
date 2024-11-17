import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from IPython.display import clear_output
np.random.seed(7)
import time
import heapq
import os

#This is the Node class the prefers higher g values in the case of a tie
class Node:
    def __init__(self, x, y, g, h):
        self.x = x  # x-coordinatez
        self.y = y  # y-coordinate
        self.g = g  # Path cost
        self.h = h  # Heuristic cost
        self.f = g + h  # Total cost
        self.b = None  # blocked? (for obstacles, 0 = free, 1 = blocked)
    def __lt__(self, other):
        if self.f == other.f:
            # If f values are the same, prioritize the one with the larger g value
            return self.g > other.g
        return self.f < other.f  # Otherwise, prioritize smaller f value

    # You may want to define a custom __repr__ for better debugging printouts.
    def __repr__(self):
        return f"Node(x={self.x}, y={self.y}, g={self.g}, h={self.h}, f={self.f})"


#This is the Node class that prefers lower g values in the case of a tie
# class Node:
#     def __init__(self, x, y, g, h):
#         self.x = x  # x-coordinate
#         self.y = y  # y-coordinate
#         self.g = g  # Path cost
#         self.h = h  # Heuristic cost
#         self.f = g + h  # Total cost
#         self.b = None  # blocked? (for obstacles, 0 = free, 1 = blocked)
#     def __lt__(self, other):
#         if self.f == other.f:
#             # If f values are the same, prioritize the one with the larger g value
#             return self.g < other.g
#         return self.f < other.f  # Otherwise, prioritize smaller f value
#
#     # You may want to define a custom __repr__ for better debugging printouts.
#     def __repr__(self):
#         return f"Node(x={self.x}, y={self.y}, g={self.g}, h={self.h}, f={self.f})"


#Function to generate the robot's view of the world
def robot_grid_generation(rows, cols):
    robot_view = [[0 for _ in range(cols)] for _ in range(rows)]
    robot_view[0][0] = 2
    robot_view[-1][-1] = 3
    return robot_view


# Function to generate a grid with obstacles
def generate_grid(rows, cols, obstacle_prob=0.3):
    # Generate a grid where 1 = obstacle, 0 = free space
    grid = np.random.choice([0, 1], size=(rows, cols), p=[1-obstacle_prob, obstacle_prob])

    # Troubleshooting: Check obstacle count
    print("Number of obstacles:", np.sum(grid))  # Number of cells with value 1 (obstacles)
    print("Total cells:", grid.size)             # Total number of cells
    print("Expected obstacle percentage:", (np.sum(grid) / grid.size) * 100, "%")

    grid[0][0] = 2  # Set the start cell as start
    grid[-1][-1] = 3  # Set the goal cell as goal

    return grid

def generate_multiple_grids(num_grids, rows, cols, obstacle_prob=0.3):
    grids = []
    for _ in range(num_grids):
        grid = generate_grid(rows, cols, obstacle_prob)
        grids.append(grid)
    return grids



# Function to visualize the grid using Matplotlib
def visualize_grid(grid, step=None):
    # Define a custom colormap: 0 = white, 1 = black, 2 = blue, 3 = green
    cmap = ListedColormap(['white', 'black', 'red', 'green'])

    # Ensure that cells with value 0 are white, 1 are black (obstacles), and 2 are blue (visited)
    plt.imshow(grid, cmap=cmap, vmin=0, vmax=3)
    plt.axis('on')
    plt.show()

    # if step is not None:
    #     plt.title(f'Step {step}')
    # plt.draw()
    # plt.pause(0.1)  # Pause to create the animation effect
    # clear_output(wait=True)

def step_by_step_visit_grid(grid):
    # Visit each cell in the grid
    step = 0
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            print(f"Visiting cell ({i}, {j})")
            if grid[i, j] == 1 or grid[i, j] == 3:
                print(f"Obstacle detected at cell ({i}, {j})")
            else:
                # Mark the visited free cell as 2 (for blue color)
                grid[i, j] = 2
                print(f"Cell ({i}, {j}) is free and marked as visited")

            # Visualize step-by-step
            step += 1
            visualize_grid(grid, step)
            time.sleep(0.05)  # Slows down the updates to make steps visible



def heuristic(x, y, x_goal, y_goal):
    # Calculate the Manhattan distance between two points
    return abs(x - x_goal) + abs(y - y_goal)

#function to find neighbors of a node
def find_neighbors(node, robot_view, closed_dict, grid):
    neighbors = []
    #define the possible directions
    directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
    for direction in directions:
        x = node.x + direction[0]
        y = node.y + direction[1]
        #check if the neighbor is within the grid
        if x >= 0 and x < grid.shape[0] and y >= 0 and y < grid.shape[1] and robot_view[x][y] != 1 and (x, y) not in closed_dict:
            neighbors.append(Node(x, y, node.g + 1, heuristic(x, y, grid.shape[0]-1, grid.shape[1]-1)))
    return neighbors


def astar(node, robot_view, grid):
    opened_list = []
    closed_dict = {}
    open_dict = {}
    robotView = robot_view

    start = node;
    open_dict[(start.x, start.y)] = start
    heapq.heappush(opened_list, start)

    while opened_list:
        current_node = heapq.heappop(opened_list)
        if (current_node.x, current_node.y) in open_dict:
            del open_dict[(current_node.x, current_node.y)]
        if (current_node.x, current_node.y) in closed_dict:
            continue
        closed_dict[(current_node.x, current_node.y)] = current_node
        if robotView[current_node.x][current_node.y] == 3:
            return closed_dict, len(closed_dict)
        moves = find_neighbors(current_node, robotView, closed_dict, grid)
        for move in moves:
            if (move.x, move.y) in closed_dict:
                continue
            if (move.x, move.y) in open_dict:
                existing_node = open_dict[(move.x, move.y)]
                if move.f < existing_node.f:
                    existing_node.g = move.g
                    existing_node.f = move.g + move.h
                    existing_node.b = current_node.b
                    heapq.heappush(opened_list, existing_node)
                    heapq.heapify(opened_list)
            else:
                heapq.heappush(opened_list, move)
                open_dict[(move.x, move.y)] = move



    return {}, len(closed_dict)

def repeatedAdaptiveAstar(grid, robot_view):
    # Define the start and goal locations
    start = Node(0, 0, 0, heuristic(0, 0, grid.shape[0] - 1, grid.shape[1] - 1))
    update_robot_view(robot_view, start, grid)
    closed_dict = {}
    count = 0
    iterationCounter = 0
    dGoal = -1
    knowledgeDict = {}
    while True:
        # Run A* to find the path based on the current robot view
        path, counts = astarAdapt(iterationCounter,knowledgeDict, dGoal, start, robot_view, grid)
        knowledgeDict = {}
        iterationCounter += 1
        for item in reversed(path.values()):
            dGoal = item.g
            break
        count = count + counts
        # print(path)
        # If no path found, return failure
        if not path:
            print("No path found ONE")
            return closed_dict, count
        for nodes in path.values():
            knowledgeDict[(nodes.x, nodes.y)] = nodes
        # Try following the path step by step
        for nodes in path.values():
            # Check if the path is blocked (1 indicates an obstacle)
            if robot_view[nodes.x][nodes.y] == 1:
                break

            # Check if the goal is reached
            if grid[nodes.x][nodes.y] == 3:
                print("Goal reached!")
                closed_dict[(nodes.x, nodes.y)] = nodes
                # print(closed_dict)
                return closed_dict, count
            iterationCounter += 1
            # Update the robot view and grid based on the current position
            update_robot_view(robot_view, nodes, grid)
            grid[nodes.x][nodes.y] = 2
            closed_dict[(nodes.x, nodes.y)] = nodes

            # Move the robot to the next step
            start = nodes
        else:
            # If we finished the loop without encountering an obstacle, goal is reached
            print("No Path found")
            # print(closed_dict)
            return closed_dict, count


def astarAdapt(steps, visitedDict, dGoal, node, robot_view, grid):
    opened_list = []
    closed_dict = {}
    open_dict = {}
    robotView = robot_view
    start = node;
    open_dict[(start.x, start.y)] = start
    heapq.heappush(opened_list, start)

    while opened_list:
        if dGoal != -1:

            current_node = heapq.heappop(opened_list)
            if (current_node.x, current_node.y) in open_dict:
                del open_dict[(current_node.x, current_node.y)]
            if (current_node.x, current_node.y) in closed_dict:
                continue
            closed_dict[(current_node.x, current_node.y)] = current_node
            if robotView[current_node.x][current_node.y] == 3:
                return closed_dict, len(closed_dict)
            moves = find_neighbors(current_node, robotView, closed_dict, grid)
            for move in moves:
                if (move.x, move.y) in closed_dict:
                    continue
                if ((move.x, move.y) not in  closed_dict and (move.x, move.y) in visitedDict):
                    if move.f <= dGoal:
                        move.h = dGoal - move.g
                        move.f = move.g + move.h
                if (move.x, move.y) in open_dict:
                    existing_node = open_dict[(move.x, move.y)]
                    if move.f < existing_node.f:
                        existing_node.g = move.g
                        existing_node.f = move.g + move.h
                        existing_node.b = current_node.b
                        heapq.heappush(opened_list, existing_node)
                        heapq.heapify(opened_list)
                else:
                    heapq.heappush(opened_list, move)
                    open_dict[(move.x, move.y)] = move
        else:

            current_node = heapq.heappop(opened_list)
            if (current_node.x, current_node.y) in open_dict:
                del open_dict[(current_node.x, current_node.y)]
            if (current_node.x, current_node.y) in closed_dict:
                continue
            closed_dict[(current_node.x, current_node.y)] = current_node
            if robotView[current_node.x][current_node.y] == 3:
                return closed_dict, len(closed_dict)
            moves = find_neighbors(current_node, robotView, closed_dict, grid)
            for move in moves:
                if (move.x, move.y) in closed_dict:
                    continue
                if (move.x, move.y) in open_dict:
                    existing_node = open_dict[(move.x, move.y)]
                    if move.f < existing_node.f:
                        existing_node.g = move.g
                        existing_node.f = move.g + move.h
                        existing_node.b = current_node.b
                        heapq.heappush(opened_list, existing_node)
                        heapq.heapify(opened_list)
                else:
                    heapq.heappush(opened_list, move)
                    open_dict[(move.x, move.y)] = move
    return {}, len(closed_dict)


def update_robot_view(robot_view, nodes, grid):
    robot_view[nodes.x][nodes.y] = 2
    directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
    for direction in directions:
        x = nodes.x + direction[0]
        y = nodes.y + direction[1]
        #check if the neighbor is within the grid
        if x >= 0 and x < grid.shape[0] and y >= 0 and y < grid.shape[1]:
            robot_view[x][y] = grid[x][y]


    return robot_view


def repeatedForwardAstar(grid, robot_view):
    # Define the start and goal locations
    start = Node(0, 0, 0, heuristic(0, 0, grid.shape[0] - 1, grid.shape[1] - 1))
    update_robot_view(robot_view, start, grid)
    closed_dict = {}
    count = 0

    while True:
        # Run A* to find the path based on the current robot view
        path, counts = astar(start, robot_view, grid)
        count = count + counts
        # print(path)
        # If no path found, return failure
        if not path:
            print("No path found ONE")
            return closed_dict, count

        # Try following the path step by step
        for nodes in path.values():
            # Check if the path is blocked (1 indicates an obstacle)
            if robot_view[nodes.x][nodes.y] == 1:
                break  # Break to recalculate the path with new knowledge

            # Check if the goal is reached
            if grid[nodes.x][nodes.y] == 3:
                print("Goal reached!")
                closed_dict[(nodes.x, nodes.y)] = nodes
                # print(closed_dict)
                return closed_dict, count

            # Update the robot view and grid based on the current position
            update_robot_view(robot_view, nodes, grid)
            grid[nodes.x][nodes.y] = 2
            closed_dict[(nodes.x, nodes.y)] = nodes

        # Move the robot to the next step
            start = nodes
        else:
            # If we finished the loop without encountering an obstacle, goal is reached
            print("No Path found")
            # print(closed_dict)
            return closed_dict, count

def find_neighbors_Backwards(node, end, robot_view, closed_dict, grid):
    neighbors = []
    #define the possible directions
    directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
    for direction in directions:
        x = node.x + direction[0]
        y = node.y + direction[1]
        # Ensure the neighbor is within bounds, not blocked, and not in closed_dict
        # Also, ensure the robot's view considers the neighbor as traversable
        if (0 <= x < grid.shape[0]) and (0 <= y < grid.shape[1]):
            if robot_view[x][y] != 1 and (x, y) not in closed_dict:
                neighbors.append(Node(x, y, node.g + 1, heuristic(x, y, end.x, end.y)))
    return neighbors

# Function to calculate all possible moves from the current position
def get_possible_moves(node, robot_view, closed_dict, grid):
    neighbors = []
    #define the possible directions
    directions = [[-1, 0], [1, 0], [0, -1], [0, 1]]
    for direction in directions:
        x = node.x + direction[0]
        y = node.y + direction[1]
        # Ensure the neighbor is within bounds, not blocked, and not in closed_dict
        # Also, ensure the robot's view considers the neighbor as traversable
        if (0 <= x < grid.shape[0]) and (0 <= y < grid.shape[1]):
            if robot_view[x][y] != 1 and (x, y) not in closed_dict:
                neighbors.append((x,y))
    neighbors
    return neighbors

def backwardsAstar(node, robot_view, grid):
    opened_list = []
    closed_dict = {}
    open_dict = {}
    robotView = robot_view
    end = node;
    start = Node(grid.shape[0]-1  , grid.shape[1]-1, 0, heuristic(grid.shape[0]-1, grid.shape[1]-1, end.x, end.y))

    open_dict[(start.x, start.y)] = start
    heapq.heappush(opened_list, start)
    #print(f"This is the start node: {end}")
    while opened_list:
        current_node = heapq.heappop(opened_list)
        if (current_node.x, current_node.y) in open_dict:
            del open_dict[(current_node.x, current_node.y)]
        if (current_node.x, current_node.y) in closed_dict:
            continue
        closed_dict[(current_node.x, current_node.y)] = current_node
        #print(closed_dict[(current_node.x, current_node.y)])
        if current_node.x == end.x and current_node.y == end.y:
            return closed_dict, len(closed_dict)
        moves = find_neighbors_Backwards(current_node, end, robotView, closed_dict, grid)
        for move in moves:
            if (move.x, move.y) in closed_dict:
                continue
            if (move.x, move.y) in open_dict:
                existing_node = open_dict[(move.x, move.y)]
                if move.f < existing_node.f:
                    existing_node.g = move.g
                    existing_node.f = move.g + move.h
                    existing_node.b = current_node.b
                    heapq.heappush(opened_list, existing_node)
                    open_dict[(move.x, move.y)] = existing_node
                    heapq.heapify(opened_list)

            else:
                heapq.heappush(opened_list, move)
                open_dict[(move.x, move.y)] = move
    return {}, len(closed_dict)
def is_valid_move(move, closed_list):
    return move in closed_list
def averages(num1,num2,num3):
    lists = []
    lists.append(num1/50.0)
    lists.append(num2/50.0)
    lists.append(num3/50.5)
    return lists
def repeatedBackwardAstar(grid, robot_view):

    # Define the start and goal locations
    start = Node(0, 0, 0, heuristic(0, 0, grid.shape[0] - 1, grid.shape[1] - 1))
    update_robot_view(robot_view, start, grid)
    closed_dict = {}
    count = 0
    possibleMoves = get_possible_moves(start, robot_view, closed_dict, grid)
    possibleMoves.append((start.x, start.y))

    while True:
        # Run A* to find the path based on the current robot view
        # print(f"Start: ({start.x}, {start.y})")
        path, counts = backwardsAstar(start, robot_view, grid)
        count = count + counts
        # print(path)
        deferredList = []

        # If no path found, return failure
        if not path:
            print("No path found")
            return closed_dict, count
        # print(len(path))
        # Try following the path step by step
        for nodes in reversed(list(path.values())):
            # print(nodes)
            # Check if the path is blocked
            if (nodes.x, nodes.y) not in possibleMoves:
                # print(f"NOT POSSIBLE ({nodes.x}, {nodes.y}), recalculating path...")
                break

            if robot_view[nodes.x][nodes.y] == 1:
                # print(f"Obstacle encountered at ({nodes.x}, {nodes.y}), recalculating path...")
                break  # Break to recalculate the path with new knowledge

            # Check if the goal is reached
            if grid[nodes.x][nodes.y] == 3:
                print("Goal reached! THREE")
                closed_dict[(nodes.x, nodes.y)] = nodes
                return closed_dict, count

            # Update the robot view and grid based on the current position
            update_robot_view(robot_view, nodes, grid)
            grid[nodes.x][nodes.y] = 2
            closed_dict[(nodes.x, nodes.y)] = nodes
            # Move the robot to the next step
            start = nodes
            for item in get_possible_moves(nodes, robot_view, closed_dict, grid):
                if item not in possibleMoves:
                    possibleMoves.append(item)
        else:
            print("No path found")
            print(closed_dict)
            return closed_dict, count



#This method is to generate a custom grid for testing purposes. NO NEED TO GRADE
def generate_custom_grid():
    # Create a 5x5 grid (default 0s, representing free spaces)
    grid = np.zeros((5, 5), dtype=int)

    # Define obstacles (1 represents obstacles)
    obstacles = [(1, 2), (2, 2), (3, 2), (3, 3), (4, 3), (2,3), (4,2)]
    for obs in obstacles:
        grid[obs] = 1

    # Mark the start 'A' at (4, 1) with a 2 and goal 'T' at (5, 5) with a 3
    grid[4, 2] = 2  # 'A' - Start
    grid[-1, -1] = 3  # 'T' - Goal

    return grid

#This method is to save the grids to a directory,
def save_grids(grids, directory="gridworlds"):
    if not os.path.exists(directory):
        os.makedirs(directory)

    for i, grid in enumerate(grids):
        filepath = os.path.join(directory, f'grid_{i}.npy')
        np.save(filepath, grid)

#This method is to load the grids from a directory,
def load_grids(directory="gridworlds"):
    grids = []
    for filename in sorted(os.listdir(directory)):
        if filename.endswith(".npy"):
            filepath = os.path.join(directory, filename)
            grids.append(np.load(filepath))
    return grids



# ========================= EVERYTHHING BELOW HERE IS FOR TESTING PURPOSES ========================= #

# rows, cols = 10, 10  # Set the size of the grid (smaller size for step-by-step)
# grid = generate_grid(rows, cols, obstacle_prob=0.3)  # Generate grid with 30% obstacles
#custom_grid1 = generate_custom_grid()  # Generate a custom grid
#custom_grid2 = generate_custom_grid()  # Generate a custom grid
# grid1 = grid.copy()
# grid2 = grid.copy()
# grid3 = grid.copy()
#
# x,count = repeatedForwardAstar(grid1, robot_grid_generation(10, 10))  # Run A* on the custom grid
# print(count)
# y, counts = repeatedBackwardAstar(grid2, robot_grid_generation(10, 10))
# print(counts)
# z, counts = repeatedAdaptiveAstar(grid3, robot_grid_generation(10, 10))
# print(counts)
#visualize_grid(grid)
# visualize_grid(grid1)
# visualize_grid(grid2)
# visualize_grid(grid3)





#BELOW IS THE PART 0 CODE, TO GENERATE 50 GRIDS AND TO RUN THE A* ALGORITHMS ON THEM

grids = generate_multiple_grids(50, 101, 101, obstacle_prob=0.3)
save_grids(grids)
grids = load_grids()
counter1 = 0
counter2 = 0
counter3 = 0
for grid in grids:
    grid1 = grid.copy()
    grid2 = grid.copy()
    grid3 = grid.copy()
    x,count = repeatedForwardAstar(grid1, robot_grid_generation(101, 101))  # Run A* on the custom grid
    counter1 += count
    y, counts = repeatedBackwardAstar(grid2, robot_grid_generation(101, 101))
    counter2 += counts
    z, countss = repeatedAdaptiveAstar(grid3, robot_grid_generation(101, 101))
    counter3 += countss
avgs = averages(counter1, counter2, counter3)
print("Average for Forward A*:", avgs[0])
print("Average for Backward A*:", avgs[1])
print("Average for Adaptive A*:", avgs[2])

#To visualize the saved grids, run the following code:
# for grid in grids:
#     visualize_grid(grid)


