from GUI import GUI
from HAL import HAL
import rospy

# global constant to define that the robot has crashed
CRASH = 1

# global constant to define robot sides
LEFT = 0
CENTRE = 1
RIGHT = 2

# time in seconds in which robots goes backward or turns
BACKING_TIME = 0.3
TURNING_TIME = 0.75

# global variables initialized
current_state = 'FORWARD'
turn_side = RIGHT
crash_side = CENTRE

# varible to define when the robot start
now_time = rospy.get_time()

def go_state(new_state):
    global current_state
    current_state = new_state

def forward2backward():
    detected_ = False

    if HAL.getBumperData().state == CRASH:
        detected_ = True
        crash_side = HAL.getBumperData().bumper

        if crash_side == LEFT:
            turn_side = 1

        elif crash_side == CENTRE:
            turn_side = 1

        elif crash_side == RIGHT:
            turn_side = -1

    return detected_

def backward2turn():
    if rospy.get_time() - now_time >= BACKING_TIME:
        return True
    return False


def turn2forward():
    if rospy.get_time() - now_time >= TURNING_TIME:
        return True
    return False



while True:
    if current_state == 'FORWARD':
        HAL.setV(2)
        HAL.setW(0)

        if forward2backward():
            go_state('BACKWARD')
            now_time = rospy.get_time()


    elif current_state == 'BACKWARD':
        HAL.setV(-2)
        HAL.setW(0)

        if backward2turn():
            go_state('TURN')
            now_time = rospy.get_time()

    elif current_state == 'TURN':
        HAL.setV(0)
        HAL.setW(2 * turn_side)

        if turn2forward():
            go_state('FORWARD')
