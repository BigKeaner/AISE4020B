#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import os

class RedTapeDetector(Node):
    def __init__(self):
        super().__init__('red_tape_detector')
        self.bridge = CvBridge()
        self.capture = cv.VideoCapture(0)
        if not self.capture.isOpened():
            self.get_logger().error("Could not open camera!")
            return
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to grab frame!")
            return

        frame = cv.resize(frame, (640, 480))
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define red color range in HSV

        lower_red1 = np.array([0,   70, 50])
        upper_red1 = np.array([15, 255, 255])
        
        lower_red2 = np.array([150, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red detection
        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask = cv.bitwise_or(mask1, mask2)

        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Draw detected contours
        for contour in contours:
            if cv.contourArea(contour) > 100:  # Minimum area threshold
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Show the results
        cv.imshow('Red Tape Detection', frame)
        cv.imshow('Mask', mask)

        if cv.waitKey(1) & 0xFF == ord('q'):
            self.capture.release()
            cv.destroyAllWindows()

def main():
    rclpy.init()
    detector = RedTapeDetector()
    rclpy.spin(detector)

if __name__ == '__main__':
    main()
