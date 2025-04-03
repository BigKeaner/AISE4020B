import turtle
import time
import cv2
import multiprocessing
from pyzbar.pyzbar import decode


# ========== BARCODE SCANNER FUNCTION ==========
# this function runs to continuously detect barcodes using OpenCV

def barcode_scanner(shared_barcode):
    camera = cv2.VideoCapture(2)
    if not camera.isOpened():
        print("⚠️ Could not open camera.")
        return

    cv2.namedWindow("Barcode Scanner Feed", cv2.WINDOW_NORMAL)
    while True:
        ret, frame = camera.read()
        if not ret:
            continue

         # detect barcodes in the frame
        barcodes = decode(frame)
        for barcode in barcodes:
            barcode_data = barcode.data.decode('utf-8')
            # barcode recognition squares and text
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text = f"{barcode_data} ({barcode.type})"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 0), 2)
            # update the shared barcode value if it's a new detection
            if barcode_data != shared_barcode.value:
                shared_barcode.value = barcode_data
                print(f"📦 Detected: {barcode_data}")
        
        # processed frame with drawings
        cv2.imshow("Barcode Scanner Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(0.3)
    
    camera.release()
    cv2.destroyAllWindows()


# ========== TURTLE SETUP AND DRAWING FUNCTIONS ==========
def setup_turtle():
    screen = turtle.Screen()
    screen.title("🏠 Smart Wheelchair Navigation System")
    screen.bgcolor("#d3d3d3")
    screen.setup(width=1000, height=700)
    return screen

def create_drawer():
    drawer = turtle.Turtle()
    drawer.speed(0)
    drawer.pensize(3)
    drawer.hideturtle()
    return drawer

def draw_colored_room(drawer, x, y, width, height, label, color):
    drawer.color("black", color)
    drawer.penup()
    drawer.goto(x, y)
    drawer.pendown()
    drawer.begin_fill()
    
    for _ in range(2):
        drawer.forward(width)
        drawer.right(90)
        drawer.forward(height)
        drawer.right(90)
    drawer.end_fill()
    drawer.penup()
    drawer.goto(x + width / 2, y - height / 2)
    drawer.write(label, align="center", font=("Arial", 12, "bold"))

def draw_midpoint(drawer, x, y, label):
    drawer.color("red")
    drawer.penup()
    drawer.goto(x - 5, y + 5)
    drawer.pendown()
    drawer.begin_fill()
    for _ in range(4):
        drawer.forward(10)
        drawer.right(90)
    drawer.end_fill()
    drawer.penup()
    drawer.goto(x, y - 5)
    drawer.color("black")
    drawer.write(label, align="center", font=("Arial", 7, "bold"))

def setup_layout(drawer):
    # Top row rooms
    draw_colored_room(drawer, -400, 250, 180, 100, "🍽 Kitchen", "#ffd966")
    draw_colored_room(drawer, -200, 250, 180, 100, "🛋 Living Room", "#f4cccc")
    draw_colored_room(drawer, 0, 250, 180, 100, "🛏 Bedroom 1", "#d9ead3")
    draw_colored_room(drawer, 200, 250, 180, 100, "🛏 Bedroom 2", "#d9ead3")

    # Second row rooms
    draw_colored_room(drawer, -400, 100, 180, 100, "🛁 Bathroom", "#cfe2f3")
    draw_colored_room(drawer, -200, 100, 180, 100, "🧼 Laundry", "#d9d2e9")
    draw_colored_room(drawer, 0, 100, 180, 100, "📖 Study", "#fff2cc")
    draw_colored_room(drawer, 200, 100, 180, 100, "🚪 Entrance", "#cccccc")

    # Bottom row rooms
    draw_colored_room(drawer, -400, -50, 180, 100, "📦 Storage", "#d3d3d3")
    draw_colored_room(drawer, -180, -50, 180, 100, "🏋️ Gym Room", "#e6b8af")
    draw_colored_room(drawer, 20, -50, 180, 100, "💼 Office", "#add8e6")

    # Corridor midpoints for top row (between top and second row)
    draw_midpoint(drawer, 290, 125, "Mid1")
    draw_midpoint(drawer, 90, 125, "Mid2")
    draw_midpoint(drawer, -110, 125, "Mid3")
    draw_midpoint(drawer, -310, 125, "Mid4")

    # Corridor midpoints for bottom row (above room doors)
    draw_midpoint(drawer, -310, -25, "Mid5")  # For Storage
    draw_midpoint(drawer, -90, -25, "Mid6")   # For Gym Room
    draw_midpoint(drawer, 110, -25, "Mid7")   # For Office

def setup_room_coords():
    # Door positions (checkpoints) on the room boundaries
    return {
        # Top row: door at bottom center (rooms span y=250 to y=150; door at y=150)
        "Kitchen": (-310, 150),      
        "Living Room": (-110, 150),     
        "Bedroom 1": (90, 150),     
        "Bedroom 2": (290, 150),   
        
        # Second row: door at top center (rooms span y=100 to y=0; door at y=100)
        "Bathroom": (-310, 100),
        "Laundry": (-110, 100),
        "Study": (90, 100),
        "Entrance": (290, 100),
        
        # Bottom row: door at top center (rooms span y=-50 to y=-150; door at y=-50)
        "Storage": (-310, -50),
        "Gym Room": (-90, -50),
        "Office": (110, -50),
        
        # Corridor midpoints
        "Mid1": (290, 125),
        "Mid2": (90, 125),
        "Mid3": (-110, 125),
        "Mid4": (-310, 125),
        "Mid5": (-310, -25),
        "Mid6": (-90, -25),
        "Mid7": (110, -25)
    }

def setup_paths():
    # navigation routes as sequences of checkpoint labels
    return {
        "Kitchen": ["Mid1", "Mid2", "Mid3", "Mid4", "Kitchen"],
        "Living Room": ["Mid1", "Mid2", "Mid3", "Living Room"],
        "Bedroom 1": ["Mid1", "Mid2", "Bedroom 1"],
        "Bedroom 2": ["Mid1", "Bedroom 2"],
        "Bathroom": ["Mid1", "Mid2", "Mid3", "Mid4", "Bathroom"],
        "Laundry": ["Mid1", "Mid2", "Mid3", "Laundry"],
        "Study": ["Mid1", "Mid2", "Study"],
        "Entrance": ["Entrance"],
        "Storage": ["Mid1", "Mid2", "Mid3", "Mid5", "Storage"],
        "Gym Room": ["Mid1", "Mid2", "Mid3", "Mid6", "Gym Room"],
        "Office": ["Mid1", "Mid2", "Mid3", "Mid7", "Office"]
    }

def ask_user_destination(screen, paths):
    valid_destinations = list(paths.keys())
    while True:
        dest = screen.textinput(
            "🚩 Destination",
            "Where do you want to go?\nOptions:\n" + ", ".join(valid_destinations)
        )
        if dest is None:
            continue
        dest = dest.strip().title()
        if dest in paths:
            print(f"🧭 Starting navigation to {dest}...")
            return dest
        else:
            print("❌ Invalid destination. Please try again.")

def run_navigation(shared_barcode):
    room_coords = setup_room_coords()
    paths = setup_paths()

    screen.addshape("wheelchair.gif")
    navigator = turtle.Turtle()
    navigator.shape("wheelchair.gif")

    navigator.color("blue")
    navigator.penup()
    navigator.goto(room_coords["Entrance"])
    navigator.pendown()
    navigator.speed(1)

    def wait_for_barcode(expected_code):
        print(f"🔎 Waiting for barcode: {expected_code}")
        while shared_barcode.value != expected_code:
            time.sleep(0.2)
        print(f"✅ Barcode matched: {expected_code}")
        shared_barcode.value = ""

    def move_to(location):
        coord = room_coords.get(location)
        if coord:
            print(f"➡️ Moving to: {location}")
            navigator.setheading(navigator.towards(coord))
            navigator.goto(coord)
            wait_for_barcode(location)

    destination = ask_user_destination(turtle.Screen(), paths)
    for stop in paths[destination]:
        move_to(stop)
    print("🎉 Navigation Complete!")
    turtle.done()

# ========== MAIN ==========
if __name__ == '__main__':
    # create a Manager and a shared value for barcode communication
    manager = multiprocessing.Manager()
    shared_barcode = manager.Value('s', "")

    #barcode scanner process
    scanner_process = multiprocessing.Process(target=barcode_scanner, args=(shared_barcode,))
    scanner_process.daemon = True
    scanner_process.start()

    # turtle graphics
    screen = setup_turtle()
    drawer = create_drawer()

    # turn off animation (for fast drawing)
    screen.tracer(0, 0)
    setup_layout(drawer)
    screen.update()

    # Re-enable automatic updates so the navigator turtle shows up & run it
    screen.tracer(1)
    run_navigation(shared_barcode)


    # terminate the scanner process once done.
    scanner_process.terminate()
