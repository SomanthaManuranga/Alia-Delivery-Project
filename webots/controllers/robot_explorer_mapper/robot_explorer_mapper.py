from controller import Robot, Supervisor
import sys
import numpy as np
import math
import heapq # For A* (not used in this explorer, but good to have if you merge)

# --- Setup ---
print("Python interpreter:", sys.executable)
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# --- Devices ---
def initialize_device_simple(name, enable_func=True, ts=timestep):
    device = robot.getDevice(name)
    if name not in ["left wheel motor", "right wheel motor"] and enable_func:
        try: # Add try-except for enabling, as some sensors might already be enabled or not exist
            device.enable(ts)
        except Exception as e:
            print(f"Note: Could not enable {name} (may already be active or not present): {e}")
    print(f"{name} initialized (or attempted)")
    return device

left_motor = initialize_device_simple("left wheel motor", enable_func=False)
right_motor = initialize_device_simple("right wheel motor", enable_func=False)
lidar = initialize_device_simple("LDS-01", enable_func=True)

if not lidar:
    print("CRITICAL: LIDAR not found or failed to initialize! This script requires LIDAR.")
    sys.exit()
if not left_motor or not right_motor:
    print("CRITICAL: Motors not found! Cannot move.")
    sys.exit()

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Robot & Map Parameters ---
MOVE_SPEED = 1.0  # Reduced speed for more careful exploration initially
TURN_SPEED = 0.5  # Reduced turn speed
CELL_SIZE = 0.25  # Size of your abstract map cells in meters
DISTANCE_THRESHOLD = CELL_SIZE * 0.3 # When robot is considered "in" a cell center
ANGLE_THRESHOLD = math.radians(5.0) # 5 degrees tolerance for alignment
print(f"CELL_SIZE: {CELL_SIZE}m, ANGLE_THRESHOLD: {math.degrees(ANGLE_THRESHOLD):.1f}°")


# Abstract Map (will be built dynamically)
MAX_MAP_ROWS = 40  # Define the maximum expected size of your explorable area
MAX_MAP_COLS = 30  # Define the maximum expected size

# Map cell values
UNKNOWN = -1
FREE = 0
OBSTACLE = 1
VISITED_AND_FREE = 2 # For cells robot center has been in and are confirmed free

# These origins map abstract_map[0][0]'s CORNER to Webots Global X,Y coordinates
X_ORIGIN_FLOOR = -5.225 # Will be calculated based on robot's start
Y_ORIGIN_FLOOR = 0.005 # Will be calculated based on robot's start

abstract_hall_map = np.full((MAX_MAP_ROWS, MAX_MAP_COLS), UNKNOWN, dtype=int)

# Define where in the abstract_hall_map array the robot's starting cell will be.
# E.g., center of the array. This helps keep array indices positive.
INITIAL_ROBOT_CELL_R = MAX_MAP_ROWS // 2
INITIAL_ROBOT_CELL_C = MAX_MAP_COLS // 2


# --- Supervisor Node & Pose ---
ROBOT_DEF_NAME = "TURTLEBOT3_BURGER"
robot_node = robot.getFromDef(ROBOT_DEF_NAME)
if not robot_node:
    print(f"CRITICAL: Robot node '{ROBOT_DEF_NAME}' not found by Supervisor.")
    sys.exit()
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")
if not trans_field or not rot_field:
    print("CRITICAL: Cannot access translation/rotation fields from Supervisor.")
    sys.exit()
print("Supervisor fields acquired.")

current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0} # Using your X,Y floor, Z-up theta convention

def get_supervisor_pose():
    """Gets pose, assuming X-Y floor, Z-up world, theta is yaw around Z."""
    p = {}
    pos_vec = trans_field.getSFVec3f() # [Global_X, Global_Y_floor, Global_Z_up]
    p["x"] = pos_vec[0]
    p["y"] = pos_vec[1] # Your floor Y is Webots Global Y

    orientation_quaternion = robot_node.getOrientation() # [qx, qy, qz, qw]
    qx, qy, qz, qw = orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3]
    
    # Yaw around Z-axis for a Z-up system (roll=0, pitch=0 after projection)
    # Standard conversion from quaternion to Euler yaw (psi) for ZYX sequence or similar Z-up
    yaw_t0 = 2.0 * (qw * qz + qx * qy)
    yaw_t1 = 1.0 - 2.0 * (qy * qy + qz * qz) # Note: this specific formula part (qy^2 + qz^2)
                                          # is for yaw if Z is the third rotation (e.g. RPY R=X, P=Y, YAW=Z).
                                          # If your robot has pitch/roll, this simple yaw might be less accurate.
                                          # For a planar robot, it's often good enough.
    theta = math.atan2(yaw_t0, yaw_t1)
    
    p["theta"] = math.atan2(math.sin(theta), math.cos(theta)) # Normalize to [-pi, pi]
    return p

# --- LIDAR & Mapping Helper Functions ---
lidar_fov = lidar.getFov()
lidar_resolution = lidar.getHorizontalResolution()
lidar_max_range = lidar.getMaxRange()
# Assuming robot's local +X is front, and theta=0 means robot faces Global +X.
# LIDAR angles are relative to robot's local +X axis.
lidar_angles_robot_frame = np.linspace(-lidar_fov / 2.0, lidar_fov / 2.0, lidar_resolution)

def cell_to_world_coords(row, col):
    """Converts abstract map cell (row, col) to world (X, Y_floor) of cell CENTER."""
    world_x = X_ORIGIN_FLOOR + (col + 0.5) * CELL_SIZE
    world_y_floor = Y_ORIGIN_FLOOR + (row + 0.5) * CELL_SIZE
    return world_x, world_y_floor

def world_to_cell_coords(world_x, world_y_floor):
    """Converts world (X, Y_floor) to abstract map cell (row, col). Clamps to map bounds."""
    col = math.floor((world_x - X_ORIGIN_FLOOR) / CELL_SIZE)
    row = math.floor((world_y_floor - Y_ORIGIN_FLOOR) / CELL_SIZE)
    # Clamp to map boundaries
    col = max(0, min(col, MAX_MAP_COLS - 1))
    row = max(0, min(row, MAX_MAP_ROWS - 1))
    return int(row), int(col)

def update_map_with_lidar(robot_current_pose_world, current_robot_cell_rc_tuple):
    """Scans with LIDAR and updates abstract_hall_map with obstacles AND FREE SPACE."""
    if lidar.getSamplingPeriod() == 0:
        # print("LIDAR not enabled for map update. Enabling now.")
        # lidar.enable(timestep) # Should be enabled at init
        # robot.step(timestep) # Allow one step for sensor to activate if just enabled
        return # Skip update if lidar was not properly ready

    ranges = lidar.getRangeImage()
    if not ranges or len(ranges) != lidar_resolution:
        print("Warning: Invalid LIDAR data received for map update.")
        return

    r_world_x = robot_current_pose_world["x"]
    r_world_y = robot_current_pose_world["y"] # Your floor Y
    r_world_theta = robot_current_pose_world["theta"]

    # Mark current cell as visited and free
    current_r, current_c = current_robot_cell_rc_tuple
    if 0 <= current_r < MAX_MAP_ROWS and 0 <= current_c < MAX_MAP_COLS:
        if abstract_hall_map[current_r, current_c] != OBSTACLE: # Don't overwrite a known obstacle
            abstract_hall_map[current_r, current_c] = VISITED_AND_FREE

    for i in range(lidar_resolution):
        scan_range = ranges[i]
        scan_angle_in_robot_frame = lidar_angles_robot_frame[i]
        
        # Absolute angle of the LIDAR ray in the world frame
        scan_angle_world = r_world_theta + scan_angle_in_robot_frame
        scan_angle_world = math.atan2(math.sin(scan_angle_world), math.cos(scan_angle_world))

        # Determine effective range (cap at lidar_max_range)
        is_obstacle_hit_on_this_ray = False
        effective_ray_length = lidar_max_range
        if scan_range is not None and not math.isinf(scan_range) and scan_range < lidar_max_range * 0.98: # Use 0.98 to avoid max range noise
            effective_ray_length = scan_range
            is_obstacle_hit_on_this_ray = True

        # Ray Casting for FREE SPACE
        # Step along the ray in small increments up to effective_ray_length
        # Mark cells as FREE if they are currently UNKNOWN
        # More precise: Bresenham. Simpler: step along ray.
        ray_step_size = CELL_SIZE / 3.0 # Step smaller than cell size for better coverage
        num_steps_in_ray = int(effective_ray_length / ray_step_size)

        for step_idx in range(1, num_steps_in_ray): # Start from step 1 to avoid robot's own cell
            dist_along_ray = step_idx * ray_step_size
            
            point_on_ray_world_x = r_world_x + dist_along_ray * math.cos(scan_angle_world)
            point_on_ray_world_y = r_world_y + dist_along_ray * math.sin(scan_angle_world)
            
            free_cell_r, free_cell_c = world_to_cell_coords(point_on_ray_world_x, point_on_ray_world_y)

            if 0 <= free_cell_r < MAX_MAP_ROWS and 0 <= free_cell_c < MAX_MAP_COLS:
                if abstract_hall_map[free_cell_r, free_cell_c] == UNKNOWN:
                    abstract_hall_map[free_cell_r, free_cell_c] = FREE
            else: # Ray went out of defined map bounds
                break 
        
        # Mark OCCUPIED cell at the end of the ray IF an obstacle was hit
        if is_obstacle_hit_on_this_ray:
            hit_point_world_x = r_world_x + effective_ray_length * math.cos(scan_angle_world)
            hit_point_world_y = r_world_y + effective_ray_length * math.sin(scan_angle_world)
            
            obstacle_cell_r, obstacle_cell_c = world_to_cell_coords(hit_point_world_x, hit_point_world_y)

            if 0 <= obstacle_cell_r < MAX_MAP_ROWS and 0 <= obstacle_cell_c < MAX_MAP_COLS:
                abstract_hall_map[obstacle_cell_r, obstacle_cell_c] = OBSTACLE

def get_target_cell_in_direction(current_cell_r, current_cell_c, absolute_world_theta):
    """Calculates the abstract cell in the direction of absolute_world_theta."""
    dr, dc = 0, 0
    # Convert world theta to simple N, S, E, W direction relative to map grid axes
    # (Assuming map rows increase with world Y, map cols increase with world X)
    if -math.pi/4 < absolute_world_theta <= math.pi/4:          dc = 1  # Facing +X world (East)
    elif math.pi/4 < absolute_world_theta <= 3*math.pi/4:       dr = 1  # Facing +Y world (South on map grid)
    elif absolute_world_theta > 3*math.pi/4 or absolute_world_theta <= -3*math.pi/4: dc = -1 # Facing -X world (West)
    else: # -3*math.pi/4 < absolute_world_theta <= -math.pi/4
        dr = -1 # Facing -Y world (North on map grid)
    
    return current_cell_r + dr, current_cell_c + dc

# --- Initialization ---
for _ in range(10): robot.step(timestep) # Let simulation and sensors stabilize
current_pose = get_supervisor_pose()

# Calculate map origins based on the robot's actual starting Supervisor pose
# This makes abstract_map[INITIAL_ROBOT_CELL_R][INITIAL_ROBOT_CELL_C]'s CENTER align with robot's start
X_ORIGIN_FLOOR = current_pose["x"] - (INITIAL_ROBOT_CELL_C + 0.5) * CELL_SIZE
Y_ORIGIN_FLOOR = current_pose["y"] - (INITIAL_ROBOT_CELL_R + 0.5) * CELL_SIZE
print(f"Calculated Map Origins: X_ORIGIN_FLOOR={X_ORIGIN_FLOOR:.3f}, Y_ORIGIN_FLOOR={Y_ORIGIN_FLOOR:.3f}")

# Determine robot's starting abstract cell after origins are set
current_robot_abstract_cell = world_to_cell_coords(current_pose["x"], current_pose["y"])
if 0 <= current_robot_abstract_cell[0] < MAX_MAP_ROWS and 0 <= current_robot_abstract_cell[1] < MAX_MAP_COLS:
    abstract_hall_map[current_robot_abstract_cell[0], current_robot_abstract_cell[1]] = VISITED_AND_FREE
else:
    print("CRITICAL: Robot started outside defined map boundaries based on origin calculation!")
    sys.exit()

print(f"Initial Supervisor Pose: x={current_pose['x']:.2f}, y={current_pose['y']:.2f}, theta={math.degrees(current_pose['theta']):.1f}°")
print(f"Robot starting in abstract cell: {current_robot_abstract_cell}")

# --- Main Control Loop ---
robot_state = "SENSE_AND_PLAN"
target_world_coords = {"x": 0.0, "y": 0.0, "theta": 0.0} # Target world pose for movement/alignment
visited_cell_history = [current_robot_abstract_cell] # Keep track to avoid short loops
stuck_counter = 0 # To detect if robot is truly stuck

loop_count = 0
# max_exploration_steps = 200 # Remove or set very high for continuous exploration

print("\n--- Robot Explorer & Mapper Started (Continuous) ---")

while robot.step(timestep) != -1: # and loop_count < max_exploration_steps :
    loop_count += 1
    prev_pose = current_pose.copy() # For checking if pose changed
    current_pose = get_supervisor_pose()
    current_robot_abstract_cell = world_to_cell_coords(current_pose["x"], current_pose["y"])

    if visited_cell_history[-1] != current_robot_abstract_cell:
        visited_cell_history.append(current_robot_abstract_cell)
        if len(visited_cell_history) > 20: # Keep a history of last 20 cells
            visited_cell_history.pop(0)
        stuck_counter = 0 # Reset stuck counter if moved to a new cell
    elif robot_state == "SENSE_AND_PLAN": # If in SENSE_AND_PLAN and didn't move to new cell
        stuck_counter +=1


    if loop_count % 60 == 0 or robot_state == "SENSE_AND_PLAN": # Print more often or on SENSE_AND_PLAN
        print(f"Time: {robot.getTime():.2f}s | State: {robot_state:<28} | CurCell: {current_robot_abstract_cell} | Pose: (x={current_pose['x']:.2f}, y={current_pose['y']:.2f}, th={math.degrees(current_pose['theta']):.1f}°)")
        if robot_state == "SENSE_AND_PLAN" and loop_count % 180 == 0: # Print map snippet occasionally during planning
             r, c = current_robot_abstract_cell
             print("Local map snippet (CurCell at center of middle row if possible):")
             for dr_offset in range(-2, 3):
                 map_r = r + dr_offset
                 if 0 <= map_r < MAX_MAP_ROWS:
                     row_str = [f"{abstract_hall_map[map_r, max(0, c_offset)]:3}" for c_offset in range(c-3, c+4) if 0 <= c_offset < MAX_MAP_COLS]
                     print(f"Row {map_r:2}: {' '.join(row_str)}")


    if robot_state == "SENSE_AND_PLAN":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        update_map_with_lidar(current_pose, current_robot_abstract_cell)

        # Exploration Strategy: Try directions in order: Front, Right, Left.
        # Prefer UNKNOWN cells, then FREE cells not recently visited.
        # Target orientation is the direction of the chosen cell.
        
        directions_to_try = [
            {"name": "TRY_FRONT", "angle_offset": 0.0},
            {"name": "TRY_RIGHT", "angle_offset": -math.pi / 2.0}, # Turn right from current orientation
            {"name": "TRY_LEFT",  "angle_offset": math.pi / 2.0},  # Turn left from current orientation
        ]
        
        chosen_next_cell = None
        chosen_target_orientation = None

        # Priority 1: Go to an UNKNOWN cell
        for direction in directions_to_try:
            potential_target_orientation = math.atan2(math.sin(current_pose["theta"] + direction["angle_offset"]), math.cos(current_pose["theta"] + direction["angle_offset"]))
            next_r, next_c = get_target_cell_in_direction(current_robot_abstract_cell[0], current_robot_abstract_cell[1], potential_target_orientation)
            
            if 0 <= next_r < MAX_MAP_ROWS and 0 <= next_c < MAX_MAP_COLS and \
               abstract_hall_map[next_r, next_c] == UNKNOWN:
                chosen_next_cell = (next_r, next_c)
                chosen_target_orientation = potential_target_orientation
                print(f"SENSE_AND_PLAN: Found UNKNOWN cell {chosen_next_cell} via {direction['name']}.")
                break
        
        # Priority 2: Go to a FREE (but not recently visited) cell
        if not chosen_next_cell:
            for direction in directions_to_try:
                potential_target_orientation = math.atan2(math.sin(current_pose["theta"] + direction["angle_offset"]), math.cos(current_pose["theta"] + direction["angle_offset"]))
                next_r, next_c = get_target_cell_in_direction(current_robot_abstract_cell[0], current_robot_abstract_cell[1], potential_target_orientation)

                if 0 <= next_r < MAX_MAP_ROWS and 0 <= next_c < MAX_MAP_COLS and \
                   (abstract_hall_map[next_r, next_c] == FREE or abstract_hall_map[next_r, next_c] == VISITED_AND_FREE) and \
                   (next_r, next_c) not in visited_cell_history[-3:]: # Avoid very short loops
                    chosen_next_cell = (next_r, next_c)
                    chosen_target_orientation = potential_target_orientation
                    print(f"SENSE_AND_PLAN: Found FREE/VISITED cell {chosen_next_cell} via {direction['name']} (not in recent history).")
                    break
        
        if chosen_next_cell:
            target_world_coords["x"], target_world_coords["y"] = cell_to_world_coords(chosen_next_cell[0], chosen_next_cell[1])
            target_world_coords["theta"] = chosen_target_orientation
            robot_state = "TURNING_TO_EXPLORE_CELL"
            print(f"  Targeting cell {chosen_next_cell} at world ({target_world_coords['x']:.2f},{target_world_coords['y']:.2f}), will align to {math.degrees(target_world_coords['theta']):.0f}°")
        else: # If no good option, try turning 180 (last resort)
            print(f"SENSE_AND_PLAN: No suitable UNKNOWN or unvisited FREE cell nearby. Planning 180 turn.")
            target_world_coords["theta"] = math.atan2(math.sin(current_pose["theta"] + math.pi), math.cos(current_pose["theta"] + math.pi))
            # Target cell remains current cell for a pure turn in place
            target_world_coords["x"], target_world_coords["y"] = cell_to_world_coords(current_robot_abstract_cell[0], current_robot_abstract_cell[1])
            robot_state = "TURNING_TO_EXPLORE_CELL"
            stuck_counter += 5 # Penalize getting stuck

        if stuck_counter > 20: # If stuck for too many cycles trying to turn/plan
            print("Robot appears to be stuck after multiple attempts. Finishing exploration.")
            robot_state = "FINISHED_EXPLORING"


    elif robot_state == "TURNING_TO_EXPLORE_CELL":
        angle_error = target_world_coords["theta"] - current_pose["theta"]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > ANGLE_THRESHOLD:
            if angle_error > 0: left_motor.setVelocity(-TURN_SPEED); right_motor.setVelocity(TURN_SPEED)
            else: left_motor.setVelocity(TURN_SPEED); right_motor.setVelocity(-TURN_SPEED)
        else:
            left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)
            print(f"TURNING_TO_EXPLORE_CELL: Aligned to {math.degrees(target_world_coords['theta']):.0f} deg.")
            
            # Check if the target cell for this orientation is different from current cell
            target_cell_for_move = world_to_cell_coords(target_world_coords["x"], target_world_coords["y"])
            if target_cell_for_move == current_robot_abstract_cell: # True if it was just a re-orientation
                robot_state = "SENSE_AND_PLAN" # Re-sense in new direction
            else:
                robot_state = "MOVING_TO_CELL" # Proceed to move to the new cell


    elif robot_state == "MOVING_TO_CELL":
        dx = target_world_coords["x"] - current_pose["x"]
        dy = target_world_coords["y"] - current_pose["y"]
        distance_to_target = math.sqrt(dx**2 + dy**2)

        desired_theta_to_target = math.atan2(dy, dx)
        angle_error_moving = desired_theta_to_target - current_pose["theta"]
        angle_error_moving = math.atan2(math.sin(angle_error_moving), math.cos(angle_error_moving))

        if abs(angle_error_moving) > ANGLE_THRESHOLD * 2.0: # If significantly off course
            print(f"MOVING_TO_CELL: Off course (error {math.degrees(angle_error_moving):.0f}°). Re-evaluating.")
            robot_state = "SENSE_AND_PLAN"
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
        elif distance_to_target > DISTANCE_THRESHOLD:
            left_motor.setVelocity(MOVE_SPEED)
            right_motor.setVelocity(MOVE_SPEED)
        else: # Arrived
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            # current_pose will update at start of next loop, then current_robot_abstract_cell
            print(f"MOVING_TO_CELL: Arrived near ({target_world_coords['x']:.2f},{target_world_coords['y']:.2f}). New cell will be {world_to_cell_coords(target_world_coords['x'], target_world_coords['y'])}")
            robot_state = "SENSE_AND_PLAN" # Sense new surroundings from new cell

    elif robot_state == "FINISHED_EXPLORING":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if loop_count % 200 == 0: print(f"Robot FINISHED EXPLORING. Idling at {current_robot_abstract_cell}")


# --- End Main Loop ---
print(f"[{robot.getTime():.2f}] Controller exiting (Simulation stopped or max loops reached).")
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Optional: Print final learned map
print("\n--- Final Explored Abstract Hall Map (0=Free, 1=Obstacle, 2=Visited, -1=Unknown) ---")
for r_idx, row_data in enumerate(abstract_hall_map):
    # Ensure row_data is an iterable of numbers for formatting
    try:
        formatted_row = [f"{int(cell):3}" for cell in row_data] # Use int() for safety if mixed types
        print(f"Row {r_idx:2}: [{' '.join(formatted_row)}]")
    except TypeError as e:
        print(f"Error formatting row {r_idx}: {row_data}. Error: {e}")