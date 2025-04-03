import turtle
import time

# Set up screen
screen = turtle.Screen()
screen.title("Wheelchair Navigation House Map")
screen.bgcolor("white")
screen.setup(width=800, height=600)

# Turtle to draw walls
drawer = turtle.Turtle()
drawer.speed(0)
drawer.pensize(3)

# Movement turtle
navigator = turtle.Turtle()
navigator.shape("turtle")
navigator.color("blue")
navigator.penup()
navigator.speed(1)

# Room centers for navigation
room_centers = {
    "Kitchen": (-275, 200),
    "Living Room": (-75, 200),
    "Bedroom 1": (125, 200),
    "Bedroom 2": (325, 200),
    "Bathroom": (-275, 50),
    "Hallway": (-75, 50),
    "Study": (125, 50),
    "Guest Room": (325, 50),
    "Entrance": (-75, -25),
}

# Function to draw a rectangle room
def draw_room(x, y, width, height, label):
    drawer.penup()
    drawer.goto(x, y)
    drawer.pendown()
    for _ in range(2):
        drawer.forward(width)
        drawer.right(90)
        drawer.forward(height)
        drawer.right(90)
    # Label the room
    drawer.penup()
    drawer.goto(x + width / 2, y - height / 2)
    drawer.write(label, align="center", font=("Arial", 12, "bold"))

# Draw rooms
draw_room(-350, 250, 150, 100, "Kitchen")
draw_room(-150, 250, 150, 100, "Living Room")
draw_room(50, 250, 150, 100, "Bedroom 1")
draw_room(250, 250, 150, 100, "Bedroom 2")
draw_room(-350, 100, 150, 100, "Bathroom")
draw_room(-150, 100, 150, 100, "Hallway")
draw_room(50, 100, 150, 100, "Study")
draw_room(250, 100, 150, 100, "Guest Room")
draw_room(-150, -50, 150, 100, "Entrance")

drawer.hideturtle()

# Move turtle to entrance to start
navigator.goto(room_centers["Entrance"])

# Read destination from barcode trigger file
def get_destination():
    try:
        with open("trigger.txt", "r") as file:
            dest = file.read().strip()
            return dest
    except FileNotFoundError:
        return ""

# Move to destination
def go_to(destination):
    if destination in room_centers:
        target = room_centers[destination]
        navigator.setheading(navigator.towards(target))
        navigator.pendown()
        navigator.goto(target)
    else:
        print(f"Invalid destination: {destination}")

# Wait briefly then navigate
time.sleep(1)
destination = get_destination()
if destination:
    go_to(destination)
else:
    print("No destination found.")

turtle.done()