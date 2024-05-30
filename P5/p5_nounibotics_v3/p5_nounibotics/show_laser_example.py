from GUI import GUI
from HAL import HAL

def main():
    # Create a HAL (robot) object
    robot = HAL()
    # Create a GUI object and link it with the robot
    gui = GUI(robot=robot)

    # robot.pose[0] = 1.1
    print(F"Robot pose: {robot.pose}")
    print(F"GUI Robot pose: {gui.getRobotPose()}")

    # Get the laser data. This comes in global world coordinates.
    laser_data = robot.getLaserData()
    # Show the laser points in the GUI
    gui.showLaser(laser_data)
    # Update the GUI (show the new image) and wait
    gui.updateGUI(block=True)

if __name__ == '__main__':
    main()
