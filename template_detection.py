#!/usr/bin/env python3
# -*-coding: utf-8 -*-
"""
Furniture detection using template matching with bounds checking
"""
import cv2 as cv
import numpy as np
import os
import time

class TemplateDetector:
    def __init__(self):
        self.templates = {
            'bedroom': [],
            'kitchen': [],
            'bathroom': [],
            'living_room': [],
            'dining_room': []
        }
        
        # Load template images
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        template_files = {
            'bedroom': 'bed.jpg',
            'kitchen': 'stove.jpg',
            'bathroom': 'shower.jpg',
            'living_room': 'couch.jpg',
            'dining_room': 'table.jpg'
        }
        
        for room, filename in template_files.items():
            path = os.path.join(template_dir, filename)
            if os.path.exists(path):
                template = cv.imread(path)
                if template is not None:
                    self.templates[room].append({
                        'image': template,
                        'size': template.shape[:2]
                    })
                    print(f"Loaded template for {room}: {filename}")
                else:
                    print(f"Failed to load template: {filename}")
            else:
                print(f"Template file not found: {filename}")

    def check_color_match(self, frame, x, y, w, h, room):
        """Check if the region matches expected colors for the room"""
        # Ensure coordinates are within frame bounds
        height, width = frame.shape[:2]
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = min(w, width - x)
        h = min(h, height - y)
        
        if w <= 0 or h <= 0:
            print("Invalid ROI dimensions")
            return False
            
        try:
            roi = frame[y:y+h, x:x+w]
            if roi.size == 0:
                print("Empty ROI")
                return False
                
            hsv_roi = cv.cvtColor(roi, cv.COLOR_BGR2HSV)
            
            if room == 'bedroom':
                mint_green_mask = cv.inRange(hsv_roi, 
                    np.array([35, 20, 100]), 
                    np.array([85, 85, 190]))
                green_ratio = cv.countNonZero(mint_green_mask) / (w * h)
                print(f"Mint green ratio: {green_ratio:.3f}")
                return green_ratio > 0.15
                
            elif room == 'bathroom' or room == 'living_room':
                mint_green_mask = cv.inRange(hsv_roi, 
                    np.array([35, 20, 100]), 
                    np.array([85, 85, 190]))
                green_ratio = cv.countNonZero(mint_green_mask) / (w * h)
                print(f"Mint green ratio: {green_ratio:.3f}")
                return green_ratio > 0.15
                
            elif room == 'kitchen':
                gray_roi = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
                white_mask = cv.inRange(gray_roi, 200, 255)
                white_ratio = cv.countNonZero(white_mask) / (w * h)
                print(f"White ratio: {white_ratio:.3f}")
                return white_ratio > 0.3
                
            elif room == 'dining_room':
                wood_mask = cv.inRange(hsv_roi,
                    np.array([10, 30, 60]),
                    np.array([30, 150, 255]))
                wood_ratio = cv.countNonZero(wood_mask) / (w * h)
                print(f"Wood ratio: {wood_ratio:.3f}")
                return wood_ratio > 0.3
                
            return True
            
        except Exception as e:
            print(f"Error in color matching: {e}")
            return False

    def detect_furniture(self, frame, room):
        """Detect furniture in frame using template matching"""
        if not self.templates[room]:
            return None
            
        height, width = frame.shape[:2]
        best_match = None
        best_val = -1
        match_size = None
        
        # Try each template
        for template_data in self.templates[room]:
            template = template_data['image']
            template_height, template_width = template_data['size']
            
            # Convert to grayscale
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            gray_template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
            
            # Try different scales
            scales = [0.5, 0.75, 1.0, 1.25, 1.5]
            for scale in scales:
                # Calculate scaled dimensions
                scaled_width = int(template_width * scale)
                scaled_height = int(template_height * scale)
                
                # Skip if scaled template is larger than frame
                if scaled_width >= width or scaled_height >= height:
                    continue
                    
                try:
                    # Resize template
                    resized_template = cv.resize(gray_template, (scaled_width, scaled_height))
                    
                    # Template matching
                    result = cv.matchTemplate(gray_frame, resized_template, cv.TM_CCOEFF_NORMED)
                    min_val, max_val, min_loc, max_loc = cv.minMaxLoc(result)
                    
                    print(f"Matching score for {room} (scale {scale:.1f}): {max_val:.3f}")
                    
                    if max_val > best_val:
                        best_val = max_val
                        best_match = max_loc
                        match_size = (scaled_width, scaled_height)
                        
                except Exception as e:
                    print(f"Error during template matching at scale {scale}: {e}")
                    continue
        
        # If we found a decent match
        if best_val > 0.4 and best_match is not None and match_size is not None:
            x, y = best_match
            w, h = match_size
            
            # Verify coordinates are within bounds
            if x >= 0 and y >= 0 and x + w <= width and y + h <= height:
                if self.check_color_match(frame, x, y, w, h, room):
                    return (x, y, w, h)
                else:
                    print(f"Color validation failed for {room}")
            else:
                print("Match coordinates out of bounds")
        
        return None

    def process_frame(self, frame, room):
        """Process frame and detect furniture"""
        if frame is None or frame.size == 0:
            print("Empty frame")
            return frame, None
            
        try:
            result = self.detect_furniture(frame, room)
            
            if result:
                x, y, w, h = result
                
                # Draw rectangle
                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate and draw center point
                center_x = x + w//2
                center_y = y + h//2
                cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Add labels
                cv.putText(frame, f"Found {room} furniture", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.putText(frame, f"Position: ({center_x}, {center_y})", (10, 60),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv.putText(frame, f"Room: {room}", (10, 90),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                return frame, (center_x, center_y)
                
        except Exception as e:
            print(f"Error in process_frame: {e}")
            
        return frame, None

def save_template(frame, room):
    """Save current frame as template"""
    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    filename = {
        'bedroom': 'bed.jpg',
        'kitchen': 'stove.jpg',
        'bathroom': 'shower.jpg',
        'living_room': 'couch.jpg',
        'dining_room': 'table.jpg'
    }[room]
    
    path = os.path.join(template_dir, filename)
    cv.imwrite(path, frame)
    print(f"Saved template for {room}: {filename}")

def main():
    # Initialize camera
    if not os.path.exists('/dev/video0'):
        print("Error: Camera device not found")
        return
        
    capture = cv.VideoCapture(0)
    if not capture.isOpened():
        print("Error: Could not open camera")
        return
    
    # Configure camera
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    capture.set(cv.CAP_PROP_FPS, 30)
    
    # Initialize detector
    detector = TemplateDetector()
    
    # Default room
    current_room = 'bedroom'
    rooms = ['bedroom', 'kitchen', 'bathroom', 'living_room', 'dining_room']
    room_index = 0
    
    print("\nControls:")
    print("- Press 'q' to quit")
    print("- Press 's' to save current frame as template")
    print("- Press 'r' to cycle through rooms")
    print(f"\nCurrently detecting: {current_room}")

    while True:
        ret, frame = capture.read()
        if not ret or frame is None:
            print("Failed to capture frame")
            continue

        try:
            # Process frame
            frame, position = detector.process_frame(frame, current_room)
            
            if position:
                print(f"Found {current_room} furniture at position: {position}")
            
            # Show frame
            cv.imshow('Template Detection', frame)
            
            # Handle keyboard input
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                save_template(frame, current_room)
            elif key == ord('r'):
                room_index = (room_index + 1) % len(rooms)
                current_room = rooms[room_index]
                print(f"\nNow detecting: {current_room}")

        except Exception as e:
            print(f"Error in main loop: {e}")
            continue

    # Cleanup
    capture.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
