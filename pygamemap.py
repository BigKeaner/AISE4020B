import pygame

pygame.init()

# Constants
GRID_SIZE = 60
GRID_WIDTH, GRID_HEIGHT = 10, 8 
WINDOW_SIZE = (GRID_WIDTH * GRID_SIZE, GRID_HEIGHT * GRID_SIZE)
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("Wheelchair Navigation Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (200, 200, 200)

# Font
font = pygame.font.SysFont('Arial', 15, bold=True)

rooms = {
    (1, 0): "Entrance", (2, 0): "Hallway",
    (1, 1): "Kitchen", (2, 1): "Bathroom",
    (1, 2): "Living Room", (2, 2): "Bedroom"
}

obstacles = [(4,1), (9,1), (9,2), (9,3)]

start_pos = (0, 0)

def draw_grid():
    for x in range(GRID_WIDTH):
        for y in range(GRID_HEIGHT):
            rect = pygame.Rect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE)
            pygame.draw.rect(window, GRAY, rect, 1)

def draw_elements():
    # Draw start position
    rect = pygame.Rect(start_pos[0]*GRID_SIZE, start_pos[1]*GRID_SIZE, GRID_SIZE, GRID_SIZE)
    pygame.draw.rect(window, GREEN, rect)

    # Draw rooms
    for pos, label in rooms.items():
        rect = pygame.Rect(pos[0]*GRID_SIZE, pos[1]*GRID_SIZE, GRID_SIZE, GRID_SIZE)
        pygame.draw.rect(window, BLUE, rect)
        text = font.render(label, True, WHITE)
        text_rect = text.get_rect(center=rect.center)
        window.blit(text, text_rect)

    # Draw obstacles
    for pos in obstacles:
        rect = pygame.Rect(pos[0]*GRID_SIZE, pos[1]*GRID_SIZE, GRID_SIZE, GRID_SIZE)
        pygame.draw.rect(window, RED, rect)

# Main loop
running = True
while running:
    window.fill(WHITE)
    draw_grid()
    draw_elements()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.update()

pygame.quit()
