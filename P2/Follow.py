from GUI import GUI
from HAL import HAL
import numpy as np
import cv2

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

# Smoothed parameters for transition
KP = (KP_S + KP_C) / 2
KI = (KI_S + KI_C) / 2
KD = (KD_S + KD_C) / 2
V_MAX = (V_MAX_S + V_MAX_C) / 2
ALPHA = (ALPHA_S + ALPHA_C) / 2

# Counter to manage transition between curve and straight segments
post_curve_steps = 0

SETPOINT = 389

prev_error = 0
integral = 0


# Compute PID controller values
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


# Compute the linear velocity based on angular velocity
def compute_velocity(v_max, alpha, w, min_v=3):
    v = v_max * (1 - alpha * abs(w))
    return np.clip(v, min_v, v_max)


# Detect red color in the image
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

# Find the largest contour in the image
def find_max_contour(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img, [max_contour], -1, (0, 255, 0), 1)

        return max_contour
    return None


# Check if a contour represents a curve
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

# Draw a bounding box around a contour
def draw_boundingbox(img, countour):
    # Calcular el cuadro delimitador alrededor del contorno mayor
    x, y, w, h = cv2.boundingRect(countour)
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)


# Find the centroid of a contour
def centroid(img, contour):
    M = cv2.moments(contour)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(SETPOINT), cy), 5, (255, 0, 0), -1)

        return cx, cy


while True:
    # Get current frame from the hardware abstraction layer (HAL)
    image = HAL.getImage()
    image_filtered = detect_red(image)

    max_contour = find_max_contour(image_filtered)

    if max_contour is not None:
        draw_boundingbox(image, max_contour)
        x, y = centroid(image, max_contour)

        # Use curve PID parameters if a curve is detected
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

        # Print and send control signals to the robot
        print(w, v)
        HAL.setV(v)
        HAL.setW(w)

    # Display the processed image using the GUI
    GUI.showImage(image)
