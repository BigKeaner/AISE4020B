import time
from localization import get_current_position
from path_planning import compute_shortest_path
from navigation import move_along_path
from obstacle_handling import check_for_obstacles
from destination import confirm_arrival

class ControlSystem:
    def __init__(self, start_node, destination_node):
        self.start_node = start_node
        self.destination_node = destination_node
        self.path = []

    def run(self):
        print("Starting navigation...")
        
        # Step 1: Localization
        current_position = get_current_position()
        print(f"Current Position: {current_position}")
        
        # Step 2: Path Planning
        self.path = compute_shortest_path(current_position, self.destination_node)
        print(f"Computed Path: {self.path}")
        
        # Step 3: Navigation & Obstacle Handling
        for node in self.path:
            print(f"Moving to node: {node}")
            move_along_path(node)
            
            # Continuously check for obstacles
            if check_for_obstacles():
                print("Obstacle detected! Recalculating path...")
                self.path = compute_shortest_path(get_current_position(), self.destination_node)
                print(f"New Path: {self.path}")
                continue
            
        # Step 4: Confirm Destination Arrival
        confirm_arrival(self.destination_node)
        print("Navigation Complete.")

if __name__ == "__main__":
    # Define start and destination nodes (replace with actual input logic)
    start = "A"
    destination = "B"
    
    control_system = ControlSystem(start, destination)
    control_system.run()
