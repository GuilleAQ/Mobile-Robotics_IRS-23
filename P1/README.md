# Basic Vacuum Cleaner

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P1/resources/figures/Basic_Vacuum_cleaner.png" alt="explode"></a> 
</div>

<h3 align="center"> Basic Vacuum Cleaner </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Task description](#Task-description)
- [First approach](#First-approach)

## Task description
The objective of this practice is to implement the logic of a navigation algorithm for an autonomous vacuum. The main objective will be to cover the largest area of ​​a house using the programmed algorithm.

#### Robot API
- from HAL import HAL - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
- from GUI import GUI - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- HAL.getBumperData().state - To establish if the robot has crashed or not. Returns a 1 if the robot collides and a 0 if it has not crashed.
- HAL.getBumperData().bumper - If the robot has crashed, it turns to 1 when the crash occurs at the center of the robot, 0 when it occurs at its left and 2 if the collision is at its right.
- HAL.getPose3d().x - to get the position of the robot (x coordinate)
- HAL.getPose3d().y - to obtain the position of the robot (y coordinate)
- HAL.getPose3d().yaw - to get the orientation of the robot with regarding the map
- HAL.getLaserData() - It allows to obtain the data of the laser sensor, which consists of 180 pairs of values ​​(0-180º, distance in millimeters).
- HAL.setV() - to set the linear speed
- HAL.setW() - to set the angular velocity

## First approach

This is a first approximation to the solution of the objective described above. 

We have a bumper and a laser as perception hardware. As actuators, we have the Roomba's motors. 

Finally, in the software section, there are no maps to make its motion decisions. We do not know, for the moment, advanced algorithms to solve the problems posed by autonomous navigation. 

That said, a basic solution but that fits our resources, may be to make a finite state machine, reactive enough for the robot to navigate safely.

### DAY 1

It is the first day that I work in practice so it occurs to me to make a code in pyhton, scalable to better solutions. It is basic but it solves the problem with a three state FSM: forward, backward and turn. Below I leave a state diagram to represent the behavior of the robot.

#### 3 FSM diagram
<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P1/resources/figures/3states_FSM_Bumper.png" alt="explode"></a> 
</div>

#### 3 FSM code

```python
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
```

As I said, the FSM is simple but I could consider the three states sufficient. 

I have used three functions that return booleans, which check whether to transition to the next state; more about them later.

#### go_state() code

The function *go_state()* receives as parameter the state to transit to and modifies the global variable *current_state*.

#### forward2backward() code

Detects if the robot has crashed and on which side, in which case it would return true.

```python
def forward2backward():
    detected = False

    if HAL.getBumperData().state == CRASH:
        detected = True
        crash_side = HAL.getBumperData().bumper

        if crash_side == LEFT:
            turn_side = 1

        elif crash_side == CENTRE:
            turn_side = 1

        elif crash_side == RIGHT:
            turn_side = -1

    return detected
```

#### backward2turn() & turn2forward() code

The diference between *backward2turn()* and *turn2forward()* is the constants *BACKING_TIME* and *TURNING_TIME*

```python
    if rospy.get_time() - now_time >= BACKING_TIME:
        return True
    return False
```
In order not to make active waiting in turn and reverse times (because of the fact that the robot could crash again), I have implemented this function that in each turn of the loop checks if the reverse time has been fulfilled and allows to continue sensing. 

It doesn't make sense to use it with the bumper because the robot doesn't have a bumper on the back so it doesn't know if it has crashed.

Nor does it make sense to use it if the turn is on itself because, as the robot is circular, if it is not crashing with anything, it will not crash with anything when it turns.

So, if they are not "useful" in this case, why implement them? The answer is simple:

- So as not to make active waits and lose reactivity.
- To make the code scalable, i.e., if the obstacle detection was done with the laser, it would be able to sense reverse and it would be useful. Or if the turn was not on itself, but advanced at the same time as it turns, it could also sense the turn time.
