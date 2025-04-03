#!/usr/bin/env python3
# -*-coding: utf-8 -*-
"""
THIS IS THE UPDATED VERSION TO USE - Added improved object detection with visualization
Features:
- Better model loading debug info
- Class name display
- Confidence scores
- Position coordinates
- Frame saving with 's' key
"""
import cv2 as cv
import numpy as np
import os
import time

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
    # Initialize detection model
    model_path = "graph_opt.pb"
    coco_names_path = "object_detection_coco.txt"
    
    print(f"Loading model from: {model_path}")
    print(f"Loading COCO names from: {coco_names_path}")

    try:
        # Load COCO class names
        with open(coco_names_path, 'r') as f:
            class_names = f.read().strip().split('\n')
        print(f"Loaded {len(class_names)} class names")

        # Load model
        model = cv.dnn.readNetFromTensorflow(model_path)
        print("Detection model loaded successfully")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # Initialize camera
    capture = initialize_camera()
    if capture is None:
        return

    print("\nControls:")
    print("- Press 'q' to quit")
    print("- Press 's' to save frame")

    while True:
        ret, frame = capture.read()
        if not ret:
            print("Failed to capture frame")
            continue

        try:
            # Prepare frame for detection (using exact parameters from working version)
            height, width = frame.shape[:2]
            blob = cv.dnn.blobFromImage(frame, 1.0, (300, 300), (127.5, 127.5, 127.5), swapRB=True)
            model.setInput(blob)
            detections = model.forward()

            # Process detections
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                
                if confidence > 0.3:  # Adjusted confidence threshold
                    # Get class ID and name
                    class_id = int(detections[0, 0, i, 1])
                    class_name = class_names[class_id - 1] if 0 <= class_id - 1 < len(class_names) else "unknown"
                    
                    # Get bounding box coordinates
                    box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                    (startX, startY, endX, endY) = box.astype("int")
                    
                    # Ensure coordinates are within frame
                    startX = max(0, min(startX, width - 1))
                    startY = max(0, min(startY, height - 1))
                    endX = max(0, min(endX, width - 1))
                    endY = max(0, min(endY, height - 1))
                    
                    # Calculate center point
                    center_x = (startX + endX) // 2
                    center_y = (startY + endY) // 2
                    
                    # Draw detection box
                    cv.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                    
                    # Draw center point
                    cv.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                    
                    # Draw labels with white background
                    label = f"{class_name}: {confidence:.2f}"
                    pos_text = f"Pos: ({center_x}, {center_y})"
                    
                    # Calculate text sizes
                    label_size = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    pos_size = cv.getTextSize(pos_text, cv.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    
                    # Draw text backgrounds
                    cv.rectangle(frame, (startX, startY - 25), (startX + label_size[0], startY), (255, 255, 255), -1)
                    cv.rectangle(frame, (startX, endY + 5), (startX + pos_size[0], endY + 30), (255, 255, 255), -1)
                    
                    # Draw text
                    cv.putText(frame, label, (startX, startY - 8),
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    cv.putText(frame, pos_text, (startX, endY + 22),
                              cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    
                    print(f"Detected {class_name} with confidence {confidence:.2f} at ({center_x}, {center_y})")

            # Show frame
            cv.imshow('Object Detection Test', frame)
            
            # Handle keyboard input
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f"detection_test_{timestamp}.jpg"
                cv.imwrite(filename, frame)
                print(f"Saved frame to {filename}")

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
