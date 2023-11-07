# Obatacle Avoidance

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P3/resources/figures/1.png" alt="explode"></a> 
</div>

<h3 align="center"> Obatacle Avoidance </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Task description](#Task-description)
- [Vector creation](#Vector-creation)
- [Situations](#Situations)
- [Video demo](#Video-demo)


## Task description
The objective of this practice is to implement the logic of the VFF navigation algorithm.

Navigation using VFF (Virtual Force Field), consists of:

  - Each object in the environment generates a repulsive force towards the robot.
  - Destiny generates an attractive force in the robot.

This makes it possible for the robot to go towards the target, distancing itself of the obstacles, so that their address is the vector sum of all the forces.


### First approach
The solution can integrate one or more of the following levels of difficulty, as well as any other one that occurs to you:

  - Go as quickly as possible
  - Choose the safest way
  - Obstacles in movement
  - Robustness in situations of indecision (zero vector sum)


### Virtual Force Field Algorithm
The Virtual Force Field Algorithm works in the following steps:
  - The robot assigns an attractive vector to the waypoint that points towards the waypoint.
  - The robot assigns a repulsive vector to the obstacle according to its sensor readings that points away from the waypoint. This is done by summing all the vectors that are translated from the sensor readings.
  - The robot follows the vector obtained by summing the target and obstacle vector.


<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P3/resources/figures/2.png" alt="explode"></a> 
</div>


## Vector creation

### Target vector
I have decided to limit the coordinates relative to the values we see in the snippet to make it more comfortable to handle the data but above all to avoid over-oscillating and unnecessary jerks towards the target.
```python
while True:
...
    # get next target
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y

    # get car coordinates and orientation
    carx = HAL.getPose3d().x
    cary = HAL.getPose3d().y
    car_yaw = HAL.getPose3d().yaw

    # get target force
    target_rel_x, target_rel_y = absolute2relative(currentTarget.getPose().x,
                                                  currentTarget.getPose().y, carx, cary, car_yaw)
    target = [np.clip(target_rel_x, 0, 2), np.clip(target_rel_y, -3, 3)]
    target_force = [target[0], target[1]]
```
### Repulsion vector
In this case I have decided not to limit the values because I need the vector to counterbalance the target vector as much as possible in risky situations because in my implementation, my robot, is very "daring". 
```python
while True:
...
    # get laser data and calculate repulsive force
    laser_data_ = HAL.getLaserData()
    parsed_laser = parse_laser_data(laser_data_)
    obs_force = get_repulsive_force(parsed_laser)
    obs_force = [obs_force[0], obs_force[1]]
```
### Average vector
I do a vector sum and multiply the scalars *alpha* and *beta* by the force
```python
while True:
...
    # get average force
    avg_x, avg_y = vectorial_sum(target_force, obs_force)
    avg_force = [avg_x * ALPHA, avg_y * BETA]
```

## Situations
### Obstacle between objective and goal
As we can see in the following image, the vector pointing to the target (green) added to the repulsion vector (red), generates a result vector (black) which, multiplied by *alpha* and *beta* scalars, describes the movement indicated to avoid the obstacle and at the same time approach the target.
<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P3/resources/figures/3.png" alt="explode"></a> 
</div>

### result vector and target vector with the same direction
In this situation the objective vector is decreasing as we approach the point and the repulsion vector is increasing, so could the local minimum of the sum vector being equal to zero be given? This question will answer below
<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P3/resources/figures/4.png" alt="explode"></a> 
</div>

#### Local minimum (sum vector=0)
Because of the way the implementation is done, one vector could cancel another one but as the velocity is only in the range of values [3, 10], when the sum vector is cancelled, it will continue to advance at a velocity of 3 until the check point is reached and the vectors are quickly adjusted again.
```python
V = np.clip(avg_force[0] , 3, 10)
W = np.clip(avg_force[1], -2.5, 2.5)
```


## Video demo
The way I have chosen the *alpha* and *beta* scalars, the robot is very "daring", that is, it is not afraid of obstacles, so it goes at a high speed and we see that it completes the circuit in **less than 100 seconds**.

[video demo]([https://urjc-my.sharepoint.com///_layouts/15/streamcumen=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview](https://urjc-my.sharepoint.com/personal/g_alcocer_2020_alumnos_urjc_es/_layouts/15/stream.aspx?id=%2Fpersonal%2Fg%5Falcocer%5F2020%5Falumnos%5Furjc%5Fes%2FDocuments%2Fobs%5Favoidance%2Emp4&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview)https://urjc-my.sharepoint.com/personal/g_alcocer_2020_alumnos_urjc_es/_layouts/15/stream.aspx?id=%2Fpersonal%2Fg%5Falcocer%5F2020%5Falumnos%5Furjc%5Fes%2FDocuments%2Fobs%5Favoidance%2Emp4&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview)
