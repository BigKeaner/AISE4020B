import cv2
import numpy as np
from pyzbar.pyzbar import decode
import os
os.environ["DYLD_LIBRARY_PATH"] = "/opt/homebrew/opt/zbar/lib"

from pyzbar.pyzbar import decode

# Open the webcam
cap = cv2.VideoCapture(0)  # 0 for default webcam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect barcodes
    barcodes = decode(gray)
    
    for barcode in barcodes:
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type

        # Get bounding box and draw rectangle
        points = np.array(barcode.polygon, np.int32)
        points = points.reshape((-1, 1, 2))
        cv2.polylines(frame, [points], isClosed=True, color=(0, 255, 0), thickness=3)

        # Put text
        cv2.putText(frame, f"{barcode_data} ({barcode_type})", 
                    (barcode.rect.left, barcode.rect.top - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        print(f"Detected: {barcode_data}, Type: {barcode_type}")

    # Display the frame
    cv2.imshow("Barcode Scanner", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
