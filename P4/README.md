# Global Navigation

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P4/resources/figures/1.png" alt="explode"></a> 
</div>

<h3 align="center"> Global Navigation </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="Status"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="License"></a>
</div>

## Table of Contents
- [Introduction](#introduction)
- [Task Description](#task-description)
- [First Approach](#first-approach)
- [Algorithm](#algorithm)
- [Setup and Installation](#setup-and-installation)
- [Usage](#usage)
- [Video Demo](#video-demo)
- [References](#references)
- [Contributors](#contributors)
- [License](#license)

## Introduction
This project focuses on developing a global navigation algorithm for autonomous cars using path planning and obstacle avoidance techniques. The goal is to navigate a car through a city map to a specified target position while avoiding obstacles.

## Task Description
The objective of this practice is to create a global navigation system that can compute an optimal path from the car's starting position to the target location on a city map. The system must handle dynamic obstacles and ensure a safe and efficient route.

## First Approach
In the initial phase, the focus was on understanding the fundamentals of path planning algorithms and setting up the simulation environment. Key tasks included:
- Familiarizing with A* and Dijkstra's algorithms.
- Setting up the simulation environment using Robotics Academy.
- Developing the initial version of the navigation algorithm.

## Algorithm
The global navigation algorithm consists of the following steps:
1. **Map Initialization**: Load the city map and initialize the cost grid.
2. **Path Planning**: Use a cost grid and a priority queue to compute the optimal path to the target.
3. **Obstacle Handling**: Increase the cost of cells around obstacles to avoid collisions.
4. **Path Execution**: Navigate the car along the computed path using the HAL module.

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P4/resources/figures/2.png" alt="explode"></a> 
</div>

### Key Components
- **Cost Grid**: A grid representing the cost of traversing each cell, used to compute the optimal path.
- **Priority Queue**: Used in the path planning algorithm to select the next cell to evaluate.
- **Obstacle Expansion**: Increases the cost around obstacles to ensure the car avoids them.
- **Path Execution**: Controls the car's movements along the planned path.

### Functions

#### compute_cost_grid
The `compute_cost_grid` function initializes a priority queue and a cost dictionary to compute the optimal path from the car's start position to the target on the map. 
It handles obstacles by increasing the cost around them.

```python
if map_array[neighbor[1], neighbor[0]] == 0:
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            nx, ny = neighbor[0] + dx, neighbor[1] + dy
            if 0 <= nx < map_array.shape[1] and 0 <= ny < map_array.shape[0]:
                map_array[ny, nx] = min(map_array[ny, nx] + 1, 255)
                visited[ny, nx] = True
```

#### get_path
The `get_path` function computes the path from the start position to the target position using the cost grid. It evaluates possible moves and selects the path with the lowest cost.


```python
for cell in neighbors:
    if 0 <= cell[0] < cost_grid.shape[1] and 0 <= cell[1] < cost_grid.shape[0]:
        cell_cost = cost_grid[cell[1], cell[0]]
        if cell_cost < lowest_cost:
            lowest_cost = cell_cost
            next_position = cell
            valid_move_found = True

```

#### navigate_to_target
The `navigate_to_target` function controls the car's movements to navigate towards the target position. It adjusts the car's speed and rotation based on the distance 
and orientation difference to the target.
```python
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

```

## Setup and Installation
To set up the project, follow these steps:
1. Clone the repository:
```sh
   git clone https://github.com/JdeRobot/RoboticsAcademy.git
   cd RoboticsAcademy/exercises/AutonomousCars/global_navigation
```

## Usage
To run the global navigation algorithm, execute the following command:

```sh
python3 global_navigation.py
```
This script initializes the cost grid, computes the path, and navigates the car to the target position using the HAL module.


## Video Demo
A video demonstration of the Monte Carlo Localization algorithm in action can be found [here](https://urjc-my.sharepoint.com/personal/g_alcocer_2020_alumnos_urjc_es/_layouts/15/stream.aspx?id=%2Fpersonal%2Fg%5Falcocer%5F2020%5Falumnos%5Furjc%5Fes%2FDocuments%2FDocumentos%2FdemoP4%2Emp4&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2E24446708%2D4d55%2D4b00%2Da180%2D927591d839f7)


## References
For more detailed information on Monte Carlo Localization and its implementation, refer to the following resources:

[JdeRobot Robotics Academy - Global Navigation](https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation/)

## Contributors
[GuilleAQ](https://github.com/GuilleAQ)

## License
This project is licensed under the Apache License. See the LICENSE file for details.
