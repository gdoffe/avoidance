#!/usr/bin/env python3

# Python modules
import cProfile
import math
import matplotlib.pyplot as plt
import os
import sys
# Get the directory of the currently running script
script_dir = os.path.dirname(os.path.abspath(__file__))
# Construct the full path relative to the script directory
module_path = os.path.join(script_dir, '../../../build/python_build/')
# Add the path to sys.path
sys.path.append(module_path)

# Custom modules
from pyavoidance import PyAvoidance, PyObstaclePolygon

# Function to draw obstacles and the path
def draw_obstacles_and_path(obstacles, path_points):
    fig, ax = plt.subplots()
    
    # Draw the obstacles
    for obstacle in obstacles:
        x_vals = [point[0] for point in obstacle]
        y_vals = [point[1] for point in obstacle]
        # Close the polygon by repeating the first point
        x_vals.append(obstacle[0][0])
        y_vals.append(obstacle[0][1])
        ax.plot(x_vals, y_vals, 'r-', label='Obstacle')

    # Draw the path if any
    if path_points:
        x_vals = [point[0] for point in path_points]
        y_vals = [point[1] for point in path_points]
        ax.plot(x_vals, y_vals, 'b--', label='Path')
    
    ax.set_xlim(0, 2000)
    ax.set_ylim(0, 3000)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Obstacle Avoidance")
    ax.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

# Define the main function to be profiled
def run_avoidance_test():
    # Define the table size (2000mm x 3000mm)
    table_borders = [
        (0, 0), (2000, 0), (2000, 3000), (0, 3000)
    ]
    
    # Create the table borders as a fixed obstacle polygon
    table = PyObstaclePolygon(table_borders)
    
    # Create the avoidance object with the table borders
    avoidance_system = PyAvoidance(table)
    
    # Define a rectangle obstacle (fixed)
    rectangle_points = [
        (400, 400), (600, 400), (600, 600), (400, 600)
    ]
    rectangle_obstacle = PyObstaclePolygon(rectangle_points)
    
    # Define a hexagon obstacle (fixed)
    hexagon_points = []
    hex_center_x, hex_center_y, hex_radius = 750, 1500, 400
    for i in range(6):
        angle = i * (3.14159 / 3)  # 60-degree angle increments
        x = hex_center_x + hex_radius * math.cos(angle)
        y = hex_center_y + hex_radius * math.sin(angle)
        hexagon_points.append((x, y))
    hexagon_obstacle = PyObstaclePolygon(hexagon_points)
    
    # Define an octagon obstacle (dynamic)
    octagon_points = []
    oct_center_x, oct_center_y, oct_radius = 1500, 2000, 400
    for i in range(8):
        angle = i * (3.14159 / 4)  # 45-degree angle increments
        x = oct_center_x + oct_radius * math.cos(angle)
        y = oct_center_y + oct_radius * math.sin(angle)
        octagon_points.append((x, y))
    octagon_obstacle = PyObstaclePolygon(octagon_points)
    
    # Add fixed obstacles to the avoidance system
    print("Adding fixed rectangle and hexagon...")
    avoidance_system.add_dynamic_obstacle(rectangle_obstacle)
    avoidance_system.add_dynamic_obstacle(hexagon_obstacle)
    
    # Add a dynamic octagon obstacle
    print("Adding dynamic octagon obstacle...")
    avoidance_system.add_dynamic_obstacle(octagon_obstacle)
    
    # Test graph building (start at (250, 250), finish at (1750, 2750))
    print("Building avoidance graph...")
    success = avoidance_system.build_graph(250, 250, 1750, 2750)
    if success:
        print("Graph built successfully!")
        path = []
        for i in range(avoidance_system.get_path_size()):
            coords = avoidance_system.get_path_pose(i)
            path.append(coords)
        print(path)
    else:
        print("Graph building failed!")
        path = []
    
    # Draw the obstacles and the computed path
    obstacles = [rectangle_points, hexagon_points, octagon_points]
    #draw_obstacles_and_path(obstacles, path)
    
    # Clear dynamic obstacles
    print("Clearing dynamic obstacles...")
    avoidance_system.clear_dynamic_obstacles()
    
    print("Test complete.")

    return obstacles, path

# Profile the run_avoidance_test function
if __name__ == "__main__":
    cProfile.run('run_avoidance_test()')
