#!/usr/bin/env python3
# -*-coding: utf-8 -*-
"""
Furniture detection for different rooms
Each room has a specific target object to detect:
- Bedroom: Bed (mint green with flower pattern)
- Kitchen: Stove (white with wooden top)
- Bathroom: Shower (wooden back with mint basin)
- Living Room: Couch (mint green with wooden arms)
- Dining Room: Table (wooden top with black legs)
"""
import cv2 as cv
import numpy as np
import os
import time

class FurnitureDetector:
    def __init__(self):
        # Calibrated color ranges
        self.mint_green = {
            'lower': np.array([37, 20, 100]),  # Calibrated values
            'upper': np.array([85, 85, 190])   # Calibrated values
        }
        self.wooden = {
            'lower': np.array([10, 30, 60]),
            'upper': np.array([30, 150, 255])
        }
        
        # Object characteristics
        self.furniture_specs = {
            'bedroom': {
                'name': 'bed',
                'primary_color': 'mint_green',
                'shape': 'rectangle',
                'pattern': 'flowers',
                'aspect_ratio': (1.0, 3.0)  # Broadened range
            },
            'kitchen': {
                'name': 'stove',
                'primary_color': 'white',
                'pattern': 'checkered',
                'aspect_ratio': (1.8, 3.2)
            },
            'bathroom': {
                'name': 'shower',
                'primary_color': 'mint_green',
                'secondary_color': 'wooden',
                'aspect_ratio': (0.6, 1.4)
            },
            'living_room': {
                'name': 'couch',
                'primary_color': 'mint_green',
                'secondary_color': 'wooden',
                'aspect_ratio': (1.8, 3.2)
            },
            'dining_room': {
                'name': 'table',
                'primary_color': 'wooden',
                'aspect_ratio': (1.3, 2.2)
            }
        }

    def detect_color(self, frame, color_range):
        """Detect areas of specific color"""
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Create mask
        mask = cv.inRange(hsv, color_range['lower'], color_range['upper'])
        
        # Clean up mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        
        return mask

    def find_objects(self, frame, room):
        """Find objects based on room specifications"""
        specs = self.furniture_specs[room]
        height, width = frame.shape[:2]
        
        # Detect primary color
        if specs['primary_color'] == 'mint_green':
            mask = self.detect_color(frame, self.mint_green)
        elif specs['primary_color'] == 'wooden':
            mask = self.detect_color(frame, self.wooden)
        else:
            # For white objects, use brightness
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            mask = cv.threshold(gray, 200, 255, cv.THRESH_BINARY)[1]
        
        # Find contours
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        best_match = None
        max_score = 0
        
        for contour in contours:
            area = cv.contourArea(contour)
            if area < 300:  # Lowered minimum area threshold
                continue
                
            # Get bounding rectangle
            x, y, w, h = cv.boundingRect(contour)
            aspect_ratio = float(w)/h
            
            # Score this contour
            score = 0
            
            # Check aspect ratio
            min_ratio, max_ratio = specs['aspect_ratio']
            if min_ratio <= aspect_ratio <= max_ratio:
                score += 1
            
            # Check relative size
            relative_size = area / (width * height)
            if 0.005 <= relative_size <= 0.5:  # Adjusted size range
                score += 1
            
            # Additional checks based on object type
            if specs['name'] == 'bed':
                # Check for flower pattern (white spots)
                roi = frame[y:y+h, x:x+w]
                gray_roi = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
                _, white_mask = cv.threshold(gray_roi, 200, 255, cv.THRESH_BINARY)
                white_ratio = cv.countNonZero(white_mask) / (w * h)
                if white_ratio > 0.01:  # Lowered threshold
                    score += 1
            
            if score > max_score:
                max_score = score
                best_match = (x, y, w, h, contour)
        
        return best_match if max_score >= 1 else None  # Lowered required score

    def process_frame(self, frame, room):
        """Process frame and detect furniture for given room"""
        result = self.find_objects(frame, room)
        
        if result:
            x, y, w, h, contour = result
            
            # Draw rectangle
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate and draw center point
            center_x = x + w//2
            center_y = y + h//2
            cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
            
            # Add labels
            object_name = self.furniture_specs[room]['name']
            cv.putText(frame, f"Found {object_name}", (10, 30),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv.putText(frame, f"Position: ({center_x}, {center_y})", (10, 60),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            return frame, (center_x, center_y)
        
        return frame, None

def initialize_camera():
    """Initialize and configure camera"""
    if not os.path.exists('/dev/video0'):
        print("Error: Camera device not found")
        return None
        
    capture = cv.VideoCapture(0)
    if not capture.isOpened():
        print("Error: Could not open camera")
        return None
        
    # Configure camera settings
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv.CAP_PROP_FPS, 30)
    
    print("Camera initialized successfully")
    return capture

def main():
    # Initialize detector
    detector = FurnitureDetector()
    
    # Initialize camera
    capture = initialize_camera()
    if capture is None:
        return

    # Default room (can be changed with keyboard)
    current_room = 'bedroom'
    rooms = ['bedroom', 'kitchen', 'bathroom', 'living_room', 'dining_room']
    room_index = 0

    print("\nControls:")
    print("- Press 'q' to quit")
    print("- Press 's' to save frame")
    print("- Press 'r' to cycle through rooms")
    print(f"\nCurrently detecting: {current_room}")

    while True:
        ret, frame = capture.read()
        if not ret:
            print("Failed to capture frame")
            continue

        try:
            # Process frame for current room
            frame, position = detector.process_frame(frame, current_room)
            
            if position:
                print(f"Found {detector.furniture_specs[current_room]['name']} at position: {position}")
            
            # Show room name
            cv.putText(frame, f"Room: {current_room}", (10, 90),
                      cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Show frame
            cv.imshow('Furniture Detection', frame)
            
            # Handle keyboard input
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"furniture_detection_{timestamp}.jpg"
                cv.imwrite(filename, frame)
                print(f"Saved frame to {filename}")
            elif key == ord('r'):
                # Cycle through rooms
                room_index = (room_index + 1) % len(rooms)
                current_room = rooms[room_index]
                print(f"\nNow detecting: {current_room}")

        except Exception as e:
            print(f"Error processing frame: {e}")
            import traceback
            traceback.print_exc()
            break

    # Cleanup
    capture.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
