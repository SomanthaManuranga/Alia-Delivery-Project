"""
Cost Map Computation using Euclidean Distance Transform

This script computes a cost map from an occupancy grid using the Euclidean
Distance Transform (EDT). The cost map represents distances from obstacles,
normalized between 0 and 1 for visualization and further processing.

Author: [Your Name]
Date: [Today's Date]
"""

from scipy.ndimage import distance_transform_edt
import numpy as np
import matplotlib.pyplot as plt

def compute_cost_map_efficient(input_grid):
    """
    Compute a cost map using the Euclidean Distance Transform.
    
    Args:
        input_grid (list of lists): Binary occupancy grid (0 = free space, 1 = obstacle)
    
    Returns:
        list of lists: Normalized cost map (0 to 1)
    """
    occupancy_grid = np.array(input_grid)  # Convert input grid to NumPy array
    cost_map = distance_transform_edt(occupancy_grid == 0)  # Compute EDT where 0 is free space

    # Normalize the cost map to a range of 0 to 1
    max_cost = np.max(cost_map)
    if max_cost > 0:
        cost_map = cost_map / max_cost  # Normalize by dividing by max value
    
    return cost_map.tolist()

# Create a 100x100 occupancy grid initialized with free space (0)
occupancy_grid_random_wall = np.zeros((100, 100))

# Set the boundary walls as obstacles (1)
occupancy_grid_random_wall[0, :] = 1  # Top boundary
occupancy_grid_random_wall[-1, :] = 1  # Bottom boundary
occupancy_grid_random_wall[:, 0] = 1  # Left boundary
occupancy_grid_random_wall[:, -1] = 1  # Right boundary

# Randomly generate a rectangular obstacle within the grid
np.random.seed(42)  # Ensure reproducibility
x_start, y_start = np.random.randint(20, 60), np.random.randint(20, 60)  # Random start position
x_size, y_size = np.random.randint(5, 15), np.random.randint(5, 15)  # Random size
occupancy_grid_random_wall[x_start:x_start + x_size, y_start:y_start + y_size] = 1  # Set obstacle

# Compute the cost map
cost_map_random_wall_efficient = compute_cost_map_efficient(occupancy_grid_random_wall)
print(cost_map_random_wall_efficient)

# Visualize the cost map
plt.imshow(cost_map_random_wall_efficient, cmap='hot', interpolation='nearest')
plt.colorbar(label='Normalized Cost')
plt.title('Normalized Cost Map (Euclidean Distance Transform)')
plt.show()
