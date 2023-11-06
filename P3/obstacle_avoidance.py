import math
from GUI import GUI
from HAL import HAL
import numpy as np

ALPHA = 20
BETA = 0.75

def parse_laser_data (laser_data):
    laser = []
    i = 0
    while i < 180:
        dist = laser_data.values[i]
        dist = min(dist, 10)
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser

def get_repulsive_force(parse_laser):
    laser = parse_laser

    laser_vectorized = []
    for dist, angle in laser:

        repulsive_x = 1/dist * math.cos(angle) * -5
        repulsive_y = 1/dist * math.sin(angle) * -15

        repulsive_cor = (repulsive_x,repulsive_y)
        laser_vectorized += [repulsive_cor]
    laser_mean = np.mean(laser_vectorized, axis=0)
    return laser_mean

def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    d_x = x_abs - robotx
    d_y = y_abs - roboty

    # Rotate with current angle
    x_rel = d_x * math.cos (-robott) - d_y * math.sin (-robott)
    y_rel = d_x * math.sin (-robott) + d_y * math.cos (-robott)

    return (x_rel, y_rel)

def vectorial_sum(v_1, v_2):
    if len(v_1) == len(v_2):
        return [x + y for x, y in zip(v_1, v_2)]
    raise ValueError("Vectors must have the same dimension")

while True:
    # get image
    image = HAL.getImage()

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

    # get laser data and calculate repulsive force
    laser_data_ = HAL.getLaserData()
    parsed_laser = parse_laser_data(laser_data_)
    obs_force = get_repulsive_force(parsed_laser)
    obs_force = [obs_force[0], obs_force[1]]

    # get average force
    avg_x, avg_y = vectorial_sum(target_force, obs_force)
    avg_force = [avg_x * ALPHA, avg_y * BETA]

    # set speed
    V = np.clip(avg_force[0] , 3, 10)
    W = np.clip(avg_force[1], -2.5, 2.5)
    print("\t\t\tV: ", V, "\tW: ", W, "")
    HAL.setV(V)
    HAL.setW(W)

    # check goal reached
    if target_rel_x <= 0:
        currentTarget.setReached(True)

    GUI.showImage(image)
    GUI.showLocalTarget(target)
    GUI.showForces(target_force, obs_force, avg_force)
