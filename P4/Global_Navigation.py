from GUI import GUI
from HAL import HAL
from MAP import MAP
import math
import queue
import numpy as np


def get_map_v24():
    map_array = MAP.getMap('/RoboticsAcademy/exercises/static/exercises/global_navigation_newmanager/resources/images/cityLargenBin.png')
    return map_array

def normalize_grid(grid):
    max_grid = np.max(grid)
    return np.clip(grid * 255 / max_grid, 0, 255).astype('uint8')

def gridToWorld(map_cell):
    world_x = map_cell[1] * 500 / 400 - 250
    world_y = map_cell[0] * 500 / 400 - 250
    return (world_x, world_y)


def compute_cost_grid(map_array, target, start):
    # Initialize the priority queue and cost dictionary
    priority_queue = queue.PriorityQueue()
    costs = {}
    visited = np.zeros(map_array.shape, dtype=bool)

    # Enqueue the target node with zero cost
    priority_queue.put((0, target))
    costs[target] = 0
    visited[target[1], target[0]] = True

    while not priority_queue.empty():
        # Get the node with the lowest cost
        current_cost, current_node = priority_queue.get()

        # Check if we've reached the start node
        if current_node == start:
            break

        # Define movement directions (8 possible moves)
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (1, -1), (-1, 1), (1, 1),
        ]

        for direction in directions:
            neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])

            # Ensure neighbor is within bounds
            if 0 <= neighbor[0] < map_array.shape[1] and 0 <= neighbor[1] < map_array.shape[0]:
                if not visited[neighbor[1], neighbor[0]]:
                    # Determine movement cost
                    if direction[0] == 0 or direction[1] == 0:
                        move_cost = current_cost + 1
                    else:
                        move_cost = current_cost + math.sqrt(2)

                    # If the neighbor is an obstacle, increase the cost around it
                    if map_array[neighbor[1], neighbor[0]] == 0:
                        for dx in range(-5, 6):
                            for dy in range(-5, 6):
                                nx, ny = neighbor[0] + dx, neighbor[1] + dy
                                if 0 <= nx < map_array.shape[1] and 0 <= ny < map_array.shape[0]:
                                    map_array[ny, nx] = min(map_array[ny, nx] + 1, 255)
                                    visited[ny, nx] = True
                        continue

                    # Update the cost map and priority queue
                    if neighbor not in costs or move_cost < costs[neighbor]:
                        costs[neighbor] = move_cost
                        priority_queue.put((move_cost, neighbor))
                        visited[neighbor[1], neighbor[0]] = True

    # Create a grid based on the computed costs
    for (x, y), cost in costs.items():
        grid[y, x] = cost

    return grid




def get_path(cost_grid, start_coords, goal_coords):
    # Initialize the route and the path
    route = []
    path = []
    current_pos = start_coords
    valid_move_found = False
    lowest_cost = float("inf")
    next_position = None

    # Compute the path from the start position to the target
    while cost_grid[current_pos[1], current_pos[0]] != 0:
        route.append(current_pos)
        # Define possible moves (neighbors)
        neighbors = [
            (current_pos[0] - 1, current_pos[1]), (current_pos[0], current_pos[1] - 1),
            (current_pos[0] + 1, current_pos[1]), (current_pos[0], current_pos[1] + 1),
            (current_pos[0] - 1, current_pos[1] - 1), (current_pos[0] + 1, current_pos[1] - 1),
            (current_pos[0] - 1, current_pos[1] + 1), (current_pos[0] + 1, current_pos[1] + 1),
        ]

        # Check each neighbor
        for cell in neighbors:
            if 0 <= cell[0] < cost_grid.shape[1] and 0 <= cell[1] < cost_grid.shape[0]:
                cell_cost = cost_grid[cell[1], cell[0]]
                if cell_cost < lowest_cost:
                    lowest_cost = cell_cost
                    next_position = cell
                    valid_move_found = True

        if valid_move_found:
            current_pos = next_position
        else:
            print("No valid neighbor found")
            break

    route.append(current_pos)

    # Initialize the first segment
    current_vector = [route[0]]
    for i in range(1, len(route)):
        current_pos = route[i]
        previous_pos = route[i - 1]
        
        if i < len(route) - 1:
            next_point = route[i + 1]
        else:
            next_point = goal_coords

        # Determine if the direction changes
        if (current_pos[0] - previous_pos[0], current_pos[1] - previous_pos[1]) != (next_point[0] - current_pos[0], next_point[1] - current_pos[1]):
            current_vector.append(current_pos)
            path.append(current_vector)
            current_vector = [current_pos]

    # Add the final segment to the goal
    current_vector.append(goal_coords)
    path.append(current_vector)

    return path




def navigate_to_target(x_goal, y_goal):
    reached = False
    while not reached:
        # Get the current position of the car
        car_pose = HAL.getPose3d()
        current_pos = [car_pose.x, car_pose.y]
        car_position_on_map = tuple(MAP.rowColumn(current_pos))
        
        # Calculate the distance to the target
        delta_x = x_goal - car_position_on_map[0]
        delta_y = y_goal - car_position_on_map[1]
        distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)

        # Calculate the correct orientation
        goal_vector_world = gridToWorld([x_goal, y_goal])
        desired_orientation = math.atan2(goal_vector_world[1] - car_pose.y, goal_vector_world[0] - car_pose.x)
        angle_diff = (desired_orientation - car_pose.yaw + math.pi) % (2 * math.pi) - math.pi

        # Adjust rotation and speed based on orientation
        if abs(angle_diff) > 0.1:
            # Rotate towards the target
            rotation_speed = max(min(angle_diff * 2.0, 1.5), -1.5)
            HAL.setW(rotation_speed)
            HAL.setV(0)
        else:
            # Move forward towards the target
            HAL.setW(0)
            if distance_to_goal > 1:
                # Set speed proportional to the distance
                speed = min(distance_to_goal * 2.0, 8.0)
                HAL.setV(speed)
                
                # Re-check orientation during motion
                car_pose = HAL.getPose3d()
                goal_vector_world = gridToWorld([x_goal, y_goal])
                desired_orientation = math.atan2(goal_vector_world[1] - car_pose.y, goal_vector_world[0] - car_pose.x)
                angle_diff = (desired_orientation - car_pose.yaw + math.pi) % (2 * math.pi) - math.pi

                # Adjust rotation if the angle difference is significant
                if abs(angle_diff) > 0.05:
                    rotation_speed = max(min(angle_diff * 0.5, 0.5), -0.5)
                    HAL.setW(rotation_speed)
                else:
                    HAL.setW(0)
            else:
                # Stop the car when the target is reached
                HAL.setV(0)
                
                reached = True

  
# Initialize the map and variables
map_array = get_map_v24()
grid = np.full(map_array.shape, 255)
car_pos = HAL.getPose3d()
pose = [car_pos.x, car_pos.y]
initial_pos = tuple(MAP.rowColumn(pose))
current_target = None
reached_target = True

while True:
    # Get the current position of the car
    car_pose = HAL.getPose3d()
    current_pos = [car_pose.x, car_pose.y]
    car_map_coords = tuple(MAP.rowColumn(current_pos))

    # Get the new target position from the GUI
    target_pose = GUI.getTargetPose()
    target_map_coords = tuple(MAP.rowColumn(target_pose))

    # Check if there is a new target
    if target_pose != current_target:
        print("New target")
        current_target = target_pose
        grid = compute_cost_grid(map_array, target_map_coords, initial_pos)

        # Check if the start position is in an obstacle
        if grid[initial_pos[1], initial_pos[0]] == 0:
            print("Target in obstacle")

        # Get the path vectors
        path_vectors = get_path(grid, initial_pos, target_map_coords)
        normalized_cost_map = normalize_grid(grid)
        GUI.showNumpy(normalized_cost_map)

        # Convert path_vectors to path_2D without using a nested list comprehension
        path_2D = []
        for segment in path_vectors:
            for x, y in segment:
                path_2D.append([x, y])

        # Show the path on the GUI
        GUI.showPath(path_2D)

        subsequent_path = path_2D[1:]

        current_index = 0
        reached_target = False

        # Navigate through the path until the target is reached
        while not reached_target:
            if current_index < len(subsequent_path):
                navigate_to_target(subsequent_path[current_index][0], subsequent_path[current_index][1])
                current_index += 1
            else:
                HAL.setV(0)  # Stop the car
                HAL.setW(0)
                reached_target = True
                print("Goal reached")
