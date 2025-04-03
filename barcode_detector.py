import os
import ctypes

ctypes.cdll.LoadLibrary("/opt/homebrew/Cellar/zbar/0.23.93_2/lib/libzbar.dylib")

import cv2
from pyzbar.pyzbar import decode

def read_barcodes(frame):
    barcodes = decode(frame)
    for barcode in barcodes:
        x, y, w, h = barcode.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        barcode_data = barcode.data.decode('utf-8')
        barcode_type = barcode.type
        text = f'{barcode_data} ({barcode_type})'
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2)
        print(f"Detected barcode: {barcode_data}")
        return barcode_data
    return None

def main():
    camera = cv2.VideoCapture(1)

    while True:
        ret, frame = camera.read()
        if not ret:
            break

        barcode = read_barcodes(frame)
        if barcode:
            print(f"Action based on: {barcode}")  # This will trigger the turtle script

        cv2.imshow('Barcode Scanner', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
