from astra_color_point import *
from astra_common import *
from astra_rgb_image import *
from calibrate_angular import *
from calibrate_linear import *
from colorHSV import *
from colorTracker import *
from common import *
from face_fllow import *
from follow_common import *
from follow_line import *
from laser_Avoidance import *
from laser_Tracker import *
from laser_Warning import *
from laserscan_to_point_publish import *
from listenline import *
from mono_Tracker import *
from multi_yahboom_joy import *
from multi_yahboom_keyboard import *
from patrol import *
from pub_image import *
from qrTracker import *  
import cv2  # Import OpenCV for image processing
from queue import *
from simple_AR import *
from singlePID import *
from stop_car import *
from test import *
from testingmovement import *
from transform_utils import *
from yahboom_joy_R2 import *
from yahboom_keyboard import *


def read_qr_code():
    qr_tracker = QR_Tracker("QR_Tracker")
    capture = cv2.VideoCapture(0)  # Capture from the default camera
    ret, frame = capture.read()  # Read a frame from the camera

    if not ret:
        print("Failed to capture image")
        return {}

    payload, _ = qr_tracker.detect_qrcode(frame)

    if payload and len(payload) == 10 and payload.isdigit():
        qr_data = { "qr_code": payload }
    else:
        qr_data = {}

    capture.release()  # Release the camera
    return qr_data

def line_tracking():
    laser_tracker = laserTracker("LaserTracker")
    capture = cv2.VideoCapture(0)  # Capture from the default camera

    while True:
        ret, frame = capture.read()  # Read a frame from the camera
        if not ret:
            print("Failed to capture image")
            break

        line_position = laser_tracker.detect_line(frame)  # Hypothetical method to detect lines
        if line_position:
            adjust_movement(line_position)  # Hypothetical method to adjust movement
        else:
            # Stop or adjust movement if no line is detected
            stop_movement()

        avoid_red_lines(frame)  # Check for red lines

    capture.release()  # Release the camera

def adjust_movement(line_position):
    # Implement logic to adjust the robot's movement based on the line position
    if line_position == "left":
        # Move left
        print("Adjusting movement to the left")
        # Add code to move the robot left
    elif line_position == "right":
        # Move right
        print("Adjusting movement to the right")
        # Add code to move the robot right
    else:
        # Move forward
        print("Moving forward")
        # Add code to move the robot forward

def stop_movement():
    # Implement logic to stop the robot's movement
    print("Stopping movement")
    # Stop movement if the tape line is no longer visible
    if not Moving:  # Check if the robot is currently moving

        print("Tape line is no longer visible, looking for nearby tapelines.")
        # Add code to search for nearby tapelines
    # Check if in waypoint finding mode and if the waypoint has been reached
    if Joy_active and Switch:

        print("Reached waypoint, stopping movement")

def avoid_red_lines(frame):
    # Implement logic to detect red lines and navigate around them
    print("Checking for red lines...")
    
    # Convert the frame to HSV color space for color detection
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range for red color in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    
    # Create a mask for red color
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    
    # Check if any red pixels are detected
    if np.any(red_mask):
        print("Red line detected! Steering around it.")
        # Logic to steer around the red line
        # For example, if the red line is detected, adjust movement
        adjust_movement("right")  # Hypothetical method to adjust movement
