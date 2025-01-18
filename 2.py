import pygame
import random
import math
import heapq

# Initialize Pygame
pygame.init()

# Screen dimensions and grid size
SCREEN_WIDTH, SCREEN_HEIGHT = 600, 600
GRID_SIZE = 20
ROWS, COLS = SCREEN_HEIGHT // GRID_SIZE, SCREEN_WIDTH // GRID_SIZE

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)  # obstacles
GREEN = (0, 255, 0)  # delivery points
RED = (255, 0, 0)  # charging stations
BLUE = (0, 0, 255)  # robots
ORANGE = (255, 165, 0)  # pick up points

# Initialize screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Multi-Robot Fleet Management Simulation")

# Clock for controlling the frame rate
clock = pygame.time.Clock()

# Entities
NUM_ROBOTS = 3
PICKUP_POINTS = [(random.randint(0, ROWS - 1), random.randint(0, COLS - 1)) for _ in range(5)]
DELIVERY_POINTS = [(random.randint(0, ROWS - 1), random.randint(0, COLS - 1)) for _ in range(5)]
CHARGING_STATIONS = [(random.randint(0, ROWS - 1), random.randint(0, COLS - 1)) for _ in range(3)]

# Obstacles
OBSTACLES = [(random.randint(0, ROWS - 1), random.randint(0, COLS - 1)) for _ in range(10)]


# Robot class
class Robot:
    def __init__(self, x, y, battery=100):
        self.x = x
        self.y = y
        self.battery = battery
        self.target = None
        self.carrying = False
        self.color = BLUE
        self.completed_journey = False
        self.stopped = False
        self.path = []  # Store the calculated path

    def move(self):
        if self.battery <= 0 or self.stopped or not self.path:
            self.stopped = True
            return

        # Follow the next step in the path
        next_step = self.path.pop(0)
        self.x, self.y = next_step
        self.battery -= 1

    def find_path(self, start, goal):
        # Implement A* algorithm
        def a_star_search(start, goal):
            open_set = []
            heapq.heappush(open_set, (0, start))
            came_from = {}
            g_score = {start: 0}
            f_score = {start: math.dist(start, goal)}

            while open_set:
                _, current = heapq.heappop(open_set)

                if current == goal:
                    path = []
                    while current in came_from:
                        path.append(current)
                        current = came_from[current]
                    path.reverse()
                    return path

                for neighbor in self.get_neighbors(current):
                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + math.dist(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
            return []

        # Get the computed path
        self.path = a_star_search(start, goal)

    def get_neighbors(self, position):
        x, y = position
        neighbors = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1)
        ]
        valid_neighbors = [
            (nx, ny) for nx, ny in neighbors
            if 0 <= nx < ROWS and 0 <= ny < COLS and (nx, ny) not in OBSTACLES
        ]
        return valid_neighbors


# Initialize robots
robots = [Robot(random.randint(0, ROWS - 1), random.randint(0, COLS - 1)) for _ in range(NUM_ROBOTS)]


# Draw the grid
def draw_grid():
    for x in range(0, SCREEN_WIDTH, GRID_SIZE):
        pygame.draw.line(screen, BLACK, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, GRID_SIZE):
        pygame.draw.line(screen, BLACK, (0, y), (SCREEN_WIDTH, y))


# Draw entities on the grid
def draw_entities():
    for x, y in PICKUP_POINTS:
        pygame.draw.rect(screen, ORANGE, (y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    for x, y in DELIVERY_POINTS:
        pygame.draw.rect(screen, GREEN, (y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    for x, y in CHARGING_STATIONS:
        pygame.draw.rect(screen, RED, (y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    for x, y in OBSTACLES:
        pygame.draw.rect(screen, BLACK, (y * GRID_SIZE, x * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    for robot in robots:
        pygame.draw.circle(screen, robot.color,
                           (robot.y * GRID_SIZE + GRID_SIZE // 2, robot.x * GRID_SIZE + GRID_SIZE // 2), GRID_SIZE // 3)
        # Display battery level
        font = pygame.font.Font(None, 20)
        battery_text = font.render(str(robot.battery), True, WHITE)
        screen.blit(battery_text, (robot.y * GRID_SIZE, robot.x * GRID_SIZE))


# Main loop
running = True
while running:
    screen.fill(WHITE)
    draw_grid()
    draw_entities()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            running = False  # Close the grid window when Enter is pressed

    # Task assignment and robot movement
    for robot in robots:
        if robot.target is None and not robot.stopped:
            # Assign nearest pickup point as the target
            if PICKUP_POINTS:
                robot.target = min(PICKUP_POINTS, key=lambda p: math.dist((robot.x, robot.y), p))
            else:
                robot.target = None

            if robot.target:
                robot.find_path((robot.x, robot.y), robot.target)

        if robot.target and not robot.stopped:
            robot.move()

            # If the robot reaches the target
            if (robot.x, robot.y) == robot.target:
                if robot.target in PICKUP_POINTS:
                    robot.carrying = True
                    PICKUP_POINTS.remove(robot.target)
                    # Assign nearest delivery point
                    if DELIVERY_POINTS:
                        robot.target = min(DELIVERY_POINTS, key=lambda p: math.dist((robot.x, robot.y), p))
                    else:
                        robot.target = None
                elif robot.target in DELIVERY_POINTS:
                    robot.carrying = False
                    DELIVERY_POINTS.remove(robot.target)
                    robot.target = None
                    robot.completed_journey = True  # Mark the journey as completed
                if robot.target:
                    robot.find_path((robot.x, robot.y), robot.target)

        # Recharge if battery is low
        if robot.battery < 20 and CHARGING_STATIONS and not robot.stopped:
            robot.target = min(CHARGING_STATIONS, key=lambda c: math.dist((robot.x, robot.y), c))
            robot.find_path((robot.x, robot.y), robot.target)
        elif robot.battery <= 0:
            robot.color = RED  # Mark the robot as inactive
            robot.stopped = True  # Mark the robot as stopped

    pygame.display.flip()
    clock.tick(10)  # Adjust frame rate

pygame.quit()