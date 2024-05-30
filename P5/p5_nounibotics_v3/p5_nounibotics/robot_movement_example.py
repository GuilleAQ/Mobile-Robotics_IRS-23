import numpy as np
import time
import MAP
from GUI import GUI
from HAL import HAL


def main():
    # Create a HAL (robot) object
    robot = HAL()
    # Set a custom initial pose
    robot.pose[0] = 1.1
    # Create a GUI object and link it with the robot
    gui = GUI(robot=robot)
    print(F"Robot pose: {gui.getRobotPose()}")

    # Set a small velocity
    robot.setV(0.3)
    robot.setW(0.8)

    while True:
        print(F"Robot pose: {gui.getRobotPose()}")
        # Get some laser data and show it in the GUI
        laser_data = robot.getLaserData()
        gui.showLaser(laser_data)

        # Robot pose is automatically updated inside the GUI "updateGUI"
        # robot.updatePose()
        gui.updateGUI()
        time.sleep(0.1)

if __name__ == '__main__':
    main()
