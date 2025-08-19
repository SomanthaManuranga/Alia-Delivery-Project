"""
3D Robotic Arm Simulator

This program simulates a 3-jointed robotic arm using Pygame.
Users can interact with the arm by clicking and dragging joints to adjust angles with the mouse.


"""

import pygame
import math

# Constants
WIDTH, HEIGHT = 800, 600  # Screen dimensions
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Robot arm parameters
link_lengths = [100, 100, 100]  # Lengths of the 3 links
joint_angles = [0, 0, 0]  # Initial joint angles (in radians)

# Pygame setup
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("3D Robotic Arm Simulator")
clock = pygame.time.Clock()

# Function to calculate forward kinematics
def forward_kinematics(angles):
    """Computes the (x, y) positions of the arm's joints based on joint angles."""
    x, y = WIDTH // 2, HEIGHT // 2  # Base of the arm at the center of the screen
    points = [(x, y)]
    for i in range(3):
        x += link_lengths[i] * math.cos(sum(angles[:i + 1]))
        y += link_lengths[i] * math.sin(sum(angles[:i + 1]))
        points.append((x, y))
    return points

# Function to display joint angles
def display_angles(screen, angles):
    """Displays the current joint angles on the screen."""
    font = pygame.font.SysFont("Arial", 20)
    for i, angle in enumerate(angles):
        text = font.render(f"Joint {i + 1}: {math.degrees(angle):.2f}°", True, BLACK)
        screen.blit(text, (10, 10 + i * 25))

# Function to calculate the angle between two points
def calculate_angle(start, end):
    """Returns the angle in radians between two points."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return math.atan2(dy, dx)

# Main loop
running = True

# Dragging state variables
dragging = False  # Whether a link is being dragged
selected_joint = -1  # Currently selected joint for adjustment

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # Check if the mouse is near any joint
            mouse_pos = pygame.mouse.get_pos()
            points = forward_kinematics(joint_angles)
            for i, point in enumerate(points):
                if math.hypot(point[0] - mouse_pos[0], point[1] - mouse_pos[1]) < 10:
                    selected_joint = i-1
                    dragging = True
                    break
        elif event.type == pygame.MOUSEBUTTONUP:
            dragging = False
            selected_joint = -1
        elif event.type == pygame.MOUSEMOTION and dragging:
            # Adjust the selected joint angle based on mouse position
            mouse_pos = pygame.mouse.get_pos()
            if selected_joint == 0:
                # Base joint rotates the entire arm
                joint_angles[0] = calculate_angle(points[0], mouse_pos)
            else:
                # Other joints rotate relative to their parent
                joint_angles[selected_joint] = calculate_angle(points[selected_joint], mouse_pos) - sum(joint_angles[:selected_joint])

    # Clear screen
    screen.fill(WHITE)

    # Draw axes
    pygame.draw.line(screen, BLACK, (WIDTH // 2, 0), (WIDTH // 2, HEIGHT), 1)
    pygame.draw.line(screen, BLACK, (0, HEIGHT // 2), (WIDTH, HEIGHT // 2), 1)

    # Compute forward kinematics
    points = forward_kinematics(joint_angles)

    # Draw links
    for i in range(3):
        start = points[i]
        end = points[i + 1]
        pygame.draw.line(screen, BLUE, start, end, 5)

    # Draw joints
    for i, point in enumerate(points):
        color = RED if i == selected_joint else BLACK
        pygame.draw.circle(screen, color, (int(point[0]), int(point[1])), 8)

    # Display joint angles
    display_angles(screen, joint_angles)

    # Update display
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
