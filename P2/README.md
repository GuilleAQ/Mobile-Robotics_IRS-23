# Visual Follow Line

<div align="center">
<img width=600px src="" alt="explode"></a> 
</div>

<h3 align="center"> Visual Follow Line </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>


## Table of Contents
- [Table of Contents](#table-of-contents)
- [Task description](#Task-description)
- [First approach](#First-approach)
- [Perception](#Perception)
- [Actuation](#Actuation)

## Task description
The goal of this exercise is to perform a PID reactive control capable of following the line painted on the racing circuit.

#### Robot API
- from HAL import HAL - to import the HAL(Hardware Abstraction Layer) library class. This class contains the functions that sends and receives information to and from the Hardware(Gazebo).
- from GUI import GUI - to import the GUI(Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
- HAL.getImage() - to get the image
- HAL.setV() - to set the linear speed
- HAL.setW() - to set the angular velocity
- GUI.showImage() - allows you to view a debug image or with relevant information

## First approach


## Perception
### DAY 1
Our first day was spent primarily understanding the requirements of the task and setting up the necessary software and tools. We experimented with various thresholds to identify the red line on the circuit. A small snippet of our initial code:

```python
def detect_red(img):
    # define the list of boundaries
    red_bajo1 = np.array([0, 100, 20], np.uint8)
    red_alto1 = np.array([8, 255, 255], np.uint8)
    red_bajo2=np.array([175, 100, 20], np.uint8)
    red_alto2=np.array([179, 255, 255], np.uint8)

    # create NumPy arrays from the boundaries
    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_red1 = cv2.inRange(frame_HSV, red_bajo1, red_alto1)
    mask_red2 = cv2.inRange(frame_HSV, red_bajo2, red_alto2)
    mask_red = cv2.add(mask_red1, mask_red2)

    output = cv2.bitwise_and(img, img, mask = mask_red)

    return output
```


### DAY 2
I have focused on finding in the filtered image the largest contour (which is then represented in a bounding box) and some strategic points such as the centroid of the contour and the point to which it has to be adjusted so that the robot is on the line.
```python
def find_max_contour(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img, [max_contour], -1, (0, 255, 0), 1)

        return max_contour
    return None
```

```python
def centroid(img, contour):
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(SETPOINT), cy), 5, (255, 0, 0), -1)

        return cx, cy
```


```python
def draw_boundingbox(img, countour):
    # Calcular el cuadro delimitador alrededor del contorno mayor
    x, y, w, h = cv2.boundingRect(countour)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
```


### DAY 3
To finalize the perception, I have implemented a function to know if the filtered contour is curved and thus to know when I am in a curve and when I am not.
```python
def is_curve(contour):
    # Calculates the perimeter of the contour
    perimeter = cv2.arcLength(contour, True)

    # Use approxPolyDP to simplify contouring
    epsilon = 0.02 * perimeter  # You can adjust this value
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Calculates the bounding box of the contour
    x, y, w, h = cv2.boundingRect(contour)

    # Calculates the aspect ratio of the bounding box
    aspect_ratio = float(w) / h

    # Calculates the area of the contour and the area/perimeter ratio
    area = cv2.contourArea(contour)
    area_perimeter_ratio = area / perimeter if perimeter != 0 else 0

    # Verifies if the contour is convex
    is_convex = cv2.isContourConvex(contour)

    if (not is_convex) and (len(approx) > 4) and (0.7 < aspect_ratio < 1.3):
        return True
    else:
        return False
```
## Actuation
### DAY 4
Implementing a function that acts as a PID controller and another one that adjusts the linear speed according to the angular speed and some maximums and minimums, I consider that it is enough to control the robot.
```python
def compute_pid(kp, ki, kd, set_point, current_value, prev_error, integral):
    # Calculate the error
    error = set_point - current_value

    # Clip integral to avoid wind-up
    max_integral = 100
    min_integral = -100
    integral += error
    integral = np.clip(integral, min_integral, max_integral)

    # Calculate derivative and clip it
    max_derivative = 100
    min_derivative = -100
    derivative = error - prev_error
    derivative = np.clip(derivative, min_derivative, max_derivative)

    # Compute output and normalize
    output = (kp * error) + (ki * integral) + (kd * derivative)
    output = output/100
    output = np.clip(output, -1, 1)

    return output, error, integral
```
The following function is taken from the internet. Alpha represents a parameter which means that the higher the value, the higher the linear speed reduction, which is good for when you are on a straight line or in a curve.
```python
def compute_velocity(v_max, alpha, w, min_v=3):
    v = v_max * (1 - alpha * abs(w))
    return np.clip(v, min_v, v_max)
```
I use this PID function for the angular velocity and depending on whether it is in a straight line or in a curve, I use some parameters or others.
```python
# straight line parameters
KP_S = 0.13
KI_S = 0.00005
KD_S = 0.83
V_MAX_S = 10.0
ALPHA_S = 0.5

# curve line parameters
KP_C = 0.53
KI_C = 0.00005
KD_C = 0.48
V_MAX_C = 6
ALPHA_C = 0.9
```
This is how I use it in the while loop
```python
# while loop
# ...
        if is_curve(max_contour):
            w, prev_error, integral = compute_pid(KP_C, KI_C, KD_C,
            SETPOINT, x, prev_error, integral)
            v = compute_velocity(V_MAX_C, ALPHA_C, w)  # Calculate v based on w

        else:
            # Use straight path PID parameters
            w, prev_error, integral = compute_pid(KP_S, KI_S, KD_S,
            SETPOINT, x, prev_error, integral)
            v = compute_velocity(V_MAX_S, ALPHA_S, w) # Calculate v based on w

        HAL.setV(v)
        HAL.setW(w)
```
### DAY 5
I have seen that when coming out of the curves it can be a bit difficult to catch the straight line speed and be on the line, so I gradually make a smooth transition from one value to another.
```python
# Smoothed parameters for transition
KP = (KP_S + KP_C) / 2
KI = (KI_S + KI_C) / 2
KD = (KD_S + KD_C) / 2
V_MAX = (V_MAX_S + V_MAX_C) / 2
ALPHA = (ALPHA_S + ALPHA_C) / 2

# Counter to manage transition between curve and straight segments
post_curve_steps = 0
```
```python
# while loop
# ...
        if is_curve(max_contour):
            w, prev_error, integral = compute_pid(KP_C, KI_C, KD_C,
            SETPOINT, x, prev_error, integral)
            v = compute_velocity(V_MAX_C, ALPHA_C, w)  # Calculate v based on w

            # Defines how many steps to wait after a curve to switch to PID from straight ahead
            post_curve_steps = 30

        elif post_curve_steps > 0:
            post_curve_steps -= 1
            w, prev_error, integral = compute_pid(KP, KI, KD, SETPOINT, x, prev_error, integral)
            v = compute_velocity(V_MAX, ALPHA_C, w) # Calculate v based on w

        else:
            # Use straight path PID parameters
            w, prev_error, integral = compute_pid(KP_S, KI_S, KD_S,
            SETPOINT, x, prev_error, integral)
            v = compute_velocity(V_MAX_S, ALPHA_S, w) # Calculate v based on w

        print(w, v)
        HAL.setV(v)
        HAL.setW(w)
```

To finish with the actuation, I would like to clarify that with two functions (one to adjust v and another to adjust w) and the appropriate parameters in each case, you can simulate **4 PIDs**, 2 for v and 2 for w. Or with an extra case (3: curve, straight and post_curve) **6 PIDs**

#### Video Demo

[simulation](https://urjc-my.sharepoint.com/:v:/r/personal/g_alcocer_2020_alumnos_urjc_es/Documents/line.mp4?csf=1&web=1&e=FFmget&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZyIsInJlZmVycmFsQXBwUGxhdGZvcm0iOiJXZWIiLCJyZWZlcnJhbE1vZGUiOiJ2aWV3In19)
