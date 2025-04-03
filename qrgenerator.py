import qrcode

# List of required labels (exact text that each QR code should contain)
labels = [
    "Mid1", "Mid2", "Mid3", "Mid4", "Mid5", "Mid6", "Mid7",
    "Kitchen", "Living Room", "Bedroom 1", "Bedroom 2",
    "Bathroom", "Laundry", "Study", "Entrance",
    "Storage", "Gym Room", "Office"
]

# Generate a QR code image for each label and save as PNG.
for label in labels:
    # Create the QR code instance
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    # Add the label data to the QR code
    qr.add_data(label)
    qr.make(fit=True)
    
    # Generate the image (black and white)
    img = qr.make_image(fill_color="black", back_color="white")
    
    # Replace spaces in the label with underscores for the filename
    filename = f"{label.replace(' ', '_')}.png"
    img.save(filename)
    print(f"Saved QR code for '{label}' as {filename}")

print("QR code generation complete.")
