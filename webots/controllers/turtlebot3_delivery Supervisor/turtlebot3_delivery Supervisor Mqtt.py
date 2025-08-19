import paho.mqtt.client as mqtt
import threading
import json
from controller import Robot, Supervisor
import sys
import numpy as np
import math
import heapq # For A* priority queue

# ===== MQTT CLIENT IMPLEMENTATION =====
# Global variables for MQTT communication
mqtt_client = None
new_command = None
command_lock = threading.Lock()

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "robot/commands"

# MQTT Callback Functions
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_TOPIC)
        print(f"Subscribed to topic: {MQTT_TOPIC}")
    else:
        print(f"Failed to connect to MQTT Broker, return code {rc}")

def on_message(client, userdata, msg):
    global new_command
    try:
        command = msg.payload.decode()
        print(f"Received MQTT command: {command}")
        
        with command_lock:
            new_command = command
            
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

def on_disconnect(client, userdata, rc):
    print("Disconnected from MQTT Broker")

def setup_mqtt():
    global mqtt_client
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.on_disconnect = on_disconnect
        
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print("MQTT client setup completed")
        return True
    except Exception as e:
        print(f"Failed to setup MQTT client: {e}")
        return False

def cleanup_mqtt():
    global mqtt_client
    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

def extract_room_from_command(command):
    """Extract room identifier from MQTT command"""
    if command.startswith("go_to_room_"):
        room_id = command.replace("go_to_room_", "")
        return f"Room{room_id.zfill(3)}"  # Convert "1" to "Room001", "101" to "Room101"
    return None

def plan_path_to_room(room_name, current_pose):
    """Plan a path to the specified room"""
    if room_name not in DELIVERY_POINTS:
        print(f"ERROR: Room '{room_name}' not found in delivery points")
        return None
    
    goal_cell_rc = DELIVERY_POINTS[room_name]
    current_cell_rc = world_to_cell_coords(current_pose["x"], current_pose["y"])
    
    print(f"Planning path from {current_cell_rc} to {room_name} at {goal_cell_rc}")
    
    # Validate cells
    if not (0 <= current_cell_rc[0] < MAP_ROWS and 0 <= current_cell_rc[1] < MAP_COLS):
        print(f"ERROR: Current position {current_cell_rc} is out of bounds")
        return None
    
    if not (0 <= goal_cell_rc[0] < MAP_ROWS and 0 <= goal_cell_rc[1] < MAP_COLS):
        print(f"ERROR: Goal position {goal_cell_rc} is out of bounds")
        return None
    
    # Find path
    cell_path = astar_search(abstract_hall_map, current_cell_rc, goal_cell_rc)
    if not cell_path:
        print(f"ERROR: No path found to {room_name}")
        return None
    
    # Convert to waypoints
    waypoints = []
    for r_cell, c_cell in cell_path:
        wp_x, wp_y_floor = cell_to_world_coords(r_cell, c_cell)
        waypoints.append((wp_x, wp_y_floor))
    
    print(f"Generated {len(waypoints)} waypoints to {room_name}")
    return waypoints
# ===== END MQTT CLIENT IMPLEMENTATION =====

# --- Setup ---
print("Python interpreter:", sys.executable)
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Initialize MQTT
mqtt_setup_success = setup_mqtt()
if not mqtt_setup_success:
    print("WARNING: MQTT setup failed, continuing without MQTT functionality")

# --- Devices ---
def initialize_device_simple(name, enable_func=True, ts=timestep):
    device = robot.getDevice(name)
    # Motors are typically not "enabled" with a timestep in the same way sensors are.
    if name not in ["left wheel motor", "right wheel motor"] and enable_func:
        device.enable(ts)
    print(f"{name} initialized")
    return device

left_motor = initialize_device_simple("left wheel motor", enable_func=False)
right_motor = initialize_device_simple("right wheel motor", enable_func=False)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Robot Parameters ---
MOVE_SPEED = 6 # Adjusted to your working value
TURN_SPEED = 2.4 # Adjusted to your working value
CELL_SIZE = 0.25 # Your cell size
DISTANCE_THRESHOLD = 0.05
ANGLE_THRESHOLD = math.radians(2.0)
print("ANGLE_THRESHOLD (radians):", ANGLE_THRESHOLD)

# --- Supervisor Node Access ---
ROBOT_DEF_NAME = "TURTLEBOT3_BURGER"
robot_node = robot.getFromDef(ROBOT_DEF_NAME)
if not robot_node:
    print(f"CRITICAL: Robot node '{ROBOT_DEF_NAME}' not found.")
    sys.exit()

trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation") # You are using this for theta
if not trans_field or not rot_field:
    print("CRITICAL: Cannot access translation/rotation fields.")
    sys.exit()
print("Supervisor fields for robot pose acquired.")

# --- Pose Getter (from your working code) ---
# This function assumes your robot navigates on the Webots Global X-Y plane,
# and Webots Global Z is its "up" direction (theta is rotation around this Z).
def get_supervisor_pose():
    current_p = {} # Local dict for current iteration's pose
    pos_vec = trans_field.getSFVec3f() # Webots [Global_X, Global_Y, Global_Z_Up]
    current_p["x"] = pos_vec[0] # Your floor X is Webots Global X
    current_p["y"] = pos_vec[1] # Your floor Y is Webots Global Y

    rot_value = rot_field.getSFRotation() # [axis_x, axis_y, axis_z_up, angle_rad]
    axis_x, axis_y, axis_z_up, angle_rad = rot_value[0], rot_value[1], rot_value[2], rot_value[3]

    theta = 0.0
    # If robot is Z-up, yaw is rotation around its local Z (which is Webots global Z)
    if abs(axis_z_up) > 0.95: # Check if Z_up is the dominant rotation axis
        theta = angle_rad * math.copysign(1.0, axis_z_up)
    else:
        # Fallback or warning if rotation isn't clearly around Z.
        # This can happen if robot is tilted or using a different world convention.
        # For simplicity, we'll try to extract something, but this might need refinement
        # based on how your specific robot model is set up in Webots.
        # A more robust method uses quaternions from robot_node.getOrientation()
        # and converts to Euler angles to get yaw around the Z-up axis.
        # However, if your getSFRotation() gives a clear axis_z_up, this is often sufficient.
        # print(f"Warning: Rotation axis not primarily Z_up. Axis:[{axis_x:.2f},{axis_y:.2f},{axis_z_up:.2f}]. Theta may be inaccurate.")
        # Using the quaternion method as a more general fallback:
        orientation_quaternion = robot_node.getOrientation() # [qx, qy, qz, qw]
        qx, qy, qz, qw = orientation_quaternion[0], orientation_quaternion[1], orientation_quaternion[2], orientation_quaternion[3]
        # Yaw around Z-axis for a Z-up system (standard Euler ZYX or similar)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz) # This uses qy, qz, implying yaw is rotation for Z axis (roll,pitch,yaw)
        theta = math.atan2(siny_cosp, cosy_cosp)

    current_p["theta"] = math.atan2(math.sin(theta), math.cos(theta)) # Normalize
    return current_p


# --- Grid Map & Pathfinding Configuration ---
# These X_ORIGIN and Y_ORIGIN are for YOUR X-Y floor plane.
# They map to Webots Global X and Webots Global Y respectively.
# You need to set these based on where you want cell (0,0) of your abstract_hall_map
# to be located in the Webots Global X-Y plane.
# EXAMPLE: If robot starts at (Webots_X=-1.85, Webots_Y=1.63) and this is center of cell (map_row=1, map_col=1)
# X_ORIGIN_FLOOR = -1.85 - (1 + 0.5) * CELL_SIZE => -1.85 - 0.75 = -2.60
# Y_ORIGIN_FLOOR =  1.63 - (1 + 0.5) * CELL_SIZE =>  1.63 - 0.75 =  0.88
X_ORIGIN_FLOOR = -14.225 # MUST BE ADJUSTED FOR YOUR WORLD AND abstract_hall_map ALIGNMENT
Y_ORIGIN_FLOOR =  -5.225 # MUST BE ADJUSTED

# This calculation should be done AFTER initial_supervisor_x, _y_floor are read
# and INTENDED_START_MAP_ROW/COL are defined.
# INTENDED_START_MAP_ROW = 38
# INTENDED_START_MAP_COL = 37
# X_ORIGIN_FLOOR = initial_supervisor_x - (INTENDED_START_MAP_COL + 0.5) * CELL_SIZE
# Y_ORIGIN_FLOOR = initial_supervisor_y_floor - (INTENDED_START_MAP_ROW + 0.5) * CELL_SIZE
# print(f"Calculated Origins: X_ORIGIN_FLOOR={X_ORIGIN_FLOOR:.3f}, Y_ORIGIN_FLOOR={Y_ORIGIN_FLOOR:.3f}")


# abstract_hall_map = [
    # [1, 1, 1, 1, 1, 1, 1],
    # [1, 0, 0, 0, 1, 0, 1], # Example: Robot starts at (1,1) -> (X=-1.85, Y=1.63) if origins above are used
    # [1, 0, 1, 0, 1, 0, 1], # Goal could be (1,5) -> cell content map[1][5] should be 0
    # [1, 0, 0, 0, 0, 0, 1],
    # [1, 1, 1, 1, 1, 1, 1]
# ]
MAP_ROWS = 61
MAP_COLS = 76 
abstract_hall_map = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 58
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 57
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 56
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 55
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 54
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 53
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 52
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 51
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 50
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 49
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 48
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 47
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 46
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 45
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 44
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 43
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 42
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 41
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 40
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 39
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 38
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 37
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 36
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 35
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 34
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 33
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 32
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 31
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 30
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 29
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Row 28
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 27
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 26
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 25
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 24
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 23
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 22
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 21
    [1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 20
    [1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 19
    [1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 18
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 17
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 16
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 15
    [1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 14
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 13
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 12
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 11
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 10
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 9
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 8
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 7
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 6
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 5
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 4
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 3
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 2
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1], # Row 1
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]  # Row 0 (This is the last row for MAP_COLS = 59)
    # ... Rows 59 to 75 would also be all 1s if they are part of the bottom border
]


# Dimensions for your abstract_hall_map
# MAP_ROWS = 40
# MAP_COLS = 13

# Create the abstract_hall_map
# abstract_hall_map = []
# for r in range(MAP_ROWS):
    # row_list = []
    # for c in range(MAP_COLS):
        # Check if it's a border cell
        # if r == 0 or r == MAP_ROWS - 1 or c == 0 or c == MAP_COLS - 1:
            # row_list.append(1)  # Obstacle (border wall)
        # else:
            # row_list.append(0)  # Free space inside
    # abstract_hall_map.append(row_list)

# --- Optional: Print the map to verify ---
# This will be a large printout for a 40x16 map.
# You might want to comment this out after verifying it once.
# print("\n--- Generated Abstract Hall Map (0=Free, 1=Obstacle) ---")
# for r_idx, row_data in enumerate(abstract_hall_map):
    # Print row number for easier reading
    # Format each number to take fixed space for better alignment
    # formatted_row = [f"{cell:2}" for cell in row_data] # Use 2 spaces per cell
    # print(f"Row {r_idx:2}: [{' '.join(formatted_row)}]")
# print("--- End of Abstract Hall Map ---\n")

# MAP_ROWS = len(abstract_hall_map)
# MAP_COLS = len(abstract_hall_map[0])

def cell_to_world_coords(row, col):
    # Converts abstract_hall_map (row, col) to Webots (Global X, Global Y) of cell center
    world_x = X_ORIGIN_FLOOR + (col + 0.5) * CELL_SIZE
    world_y_floor = Y_ORIGIN_FLOOR + (row + 0.5) * CELL_SIZE # This is Webots Global Y
    return world_x, world_y_floor

def world_to_cell_coords(world_x, world_y_floor):
    # Converts Webots (Global X, Global Y) to abstract_hall_map (row, col)
    col = math.floor((world_x - X_ORIGIN_FLOOR) / CELL_SIZE)
    row = math.floor((world_y_floor - Y_ORIGIN_FLOOR) / CELL_SIZE)
    col = max(0, min(col, MAP_COLS - 1))
    row = max(0, min(row, MAP_ROWS - 1))
    return int(row), int(col)

# --- A* Pathfinding Algorithm ---
class AStarNode:
    def __init__(self, parent=None, position=None):
        self.parent = parent; self.position = position
        self.g = 0; self.h = 0; self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f
    def __hash__(self): return hash(self.position)

def heuristic(node_pos, goal_pos): # Manhattan distance
    return abs(node_pos[0] - goal_pos[0]) + abs(node_pos[1] - goal_pos[1])

def astar_search(grid_map_data, start_rc, goal_rc):
    rows, cols = len(grid_map_data), len(grid_map_data[0])
    start_node, goal_node = AStarNode(None, start_rc), AStarNode(None, goal_rc)
    open_list, closed_set = [], set()
    heapq.heappush(open_list, start_node)
    movements = [(0, -1), (0, 1), (-1, 0), (1, 0)] # N, E, S, W

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)
        if current_node.position == goal_node.position:
            path = []; temp = current_node
            while temp: path.append(temp.position); temp = temp.parent
            return path[::-1]
        children = []
        for move in movements:
            node_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])
            if not (0 <= node_pos[0] < rows and 0 <= node_pos[1] < cols and grid_map_data[node_pos[0]][node_pos[1]] == 0):
                continue
            children.append(AStarNode(current_node, node_pos))
        for child in children:
            if child.position in closed_set: continue
            child.g = current_node.g + 1
            child.h = heuristic(child.position, goal_node.position)
            child.f = child.g + child.h
            if not any(open_node for open_node in open_list if child == open_node and child.g >= open_node.g):
                heapq.heappush(open_list, child)
    return None

# --- Initialization with Pathfinding ---
for _ in range(5): robot.step(timestep) # Settle
current_pose = get_supervisor_pose() # Use your definition of x,y,theta for current_pose dict
initial_supervisor_x = current_pose["x"]
initial_supervisor_y_floor = current_pose["y"] # This is Webots Global Y
print(f"Initial Supervisor Pose: x={initial_supervisor_x:.2f}, y_floor={initial_supervisor_y_floor:.2f}, theta={math.degrees(current_pose['theta']):.1f}°")

start_cell_rc = world_to_cell_coords(initial_supervisor_x, initial_supervisor_y_floor)
print(f"Robot starting in abstract cell: {start_cell_rc}")

# --- Named Delivery Points and Goal Setting ---
DELIVERY_POINTS = {
    "Room001": (26,46),   # Support room commands like go_to_room_1
    "Room002": (43, 48),
    "Room003": (30, 22),
    "Room101": (26,46),   # Support room commands like go_to_room_101
    "Room102": (43, 48),
    "Room103": (30, 22),
    # Add more rooms as needed
}

#_CHOOSE_YOUR_GOAL_HERE_
chosen_goal_name = "Room103" # Change this to test different destinations

if chosen_goal_name in DELIVERY_POINTS:
    goal_cell_rc = DELIVERY_POINTS[chosen_goal_name]
    print(f"Targeting delivery point: '{chosen_goal_name}' at cell {goal_cell_rc}")
else:
    print(f"CRITICAL: Unknown delivery point name '{chosen_goal_name}'. Using default or exiting.")
    # Fallback goal or exit
    goal_cell_rc = (MAP_ROWS -2, MAP_COLS -2) # Default if name not found (make sure it's valid)
    # sys.exit()

if not (0 <= start_cell_rc[0] < MAP_ROWS and 0 <= start_cell_rc[1] < MAP_COLS and abstract_hall_map[start_cell_rc[0]][start_cell_rc[1]] == 0):
    print(f"CRITICAL: Start cell {start_cell_rc} is invalid (out of bounds or obstacle). Check X/Y_ORIGIN_FLOOR or map definition.")
    sys.exit()
if not (0 <= goal_cell_rc[0] < MAP_ROWS and 0 <= goal_cell_rc[1] < MAP_COLS and abstract_hall_map[goal_cell_rc[0]][goal_cell_rc[1]] == 0):
    print(f"CRITICAL: Goal cell {goal_cell_rc} is invalid. Choose a valid free cell.")
    sys.exit()

cell_path = astar_search(abstract_hall_map, start_cell_rc, goal_cell_rc)
WAYPOINTS = []
if cell_path:
    print(f"A* Path (cells): {cell_path}")
    for r_cell, c_cell in cell_path:
        # The path from A* includes the start cell.
        # Our movement controller moves TO waypoints.
        # If the robot is already *at* the center of the first cell in cell_path,
        # it will quickly mark it done.
        wp_x, wp_y_floor = cell_to_world_coords(r_cell, c_cell)
        WAYPOINTS.append((wp_x, wp_y_floor))
    print(f"Generated WAYPOINTS (world x, y_floor): {WAYPOINTS}")
else:
    print("No path found by A* to the goal cell.")

current_waypoint_index = 0
robot_state = "IDLE" if WAYPOINTS else "FINISHED"
target_pose = {"x": 0.0, "y": 0.0} # Will use 'y' as your floor Y

print("\n--- ALIA-DELIVERY Robot Controller (Pathfinding on X-Y Floor) ---")
if WAYPOINTS: print(f"Navigating {len(WAYPOINTS)} waypoints generated by A*.")

# --- Main Control Loop ---
loop_count = 0
while robot.step(timestep) != -1:
    loop_count += 1
    current_pose = get_supervisor_pose() # Updates current_pose with "x", "y", "theta"

    # ===== MQTT COMMAND PROCESSING =====
    with command_lock:
        if new_command:
            print(f"\n=== Processing MQTT Command: {new_command} ===")
            
            room_name = extract_room_from_command(new_command)
            if room_name:
                print(f"Extracted room: {room_name}")
                
                # Plan new path
                new_waypoints = plan_path_to_room(room_name, current_pose)
                
                if new_waypoints:
                    # Stop current movement and start new mission
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    
                    WAYPOINTS = new_waypoints
                    current_waypoint_index = 0
                    robot_state = "IDLE"
                    
                    print(f"New mission started: Going to {room_name} with {len(WAYPOINTS)} waypoints")
                else:
                    print(f"Failed to plan path to {room_name}")
            else:
                print(f"Could not extract room from command: {new_command}")
            
            new_command = None  # Clear the command
    # ===== END MQTT COMMAND PROCESSING =====

    # Periodic debug print
    if loop_count % 50 == 0 : # Print less frequently
        if robot_state != "FINISHED" and current_waypoint_index < len(WAYPOINTS):
            current_target_for_print = WAYPOINTS[current_waypoint_index]
            print(f"[{robot.getTime():.2f}] State:{robot_state} | Cur:(x={current_pose['x']:.2f},y={current_pose['y']:.2f},θ={math.degrees(current_pose['theta']):.1f}°) "
                  f"| TargWP_idx:{current_waypoint_index} (x={current_target_for_print[0]:.2f},y={current_target_for_print[1]:.2f})")
        elif robot_state == "FINISHED":
            print(f"[{robot.getTime():.2f}] State: FINISHED. Robot idle at (x={current_pose['x']:.2f},y={current_pose['y']:.2f})")


    if robot_state == "IDLE":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if current_waypoint_index < len(WAYPOINTS):
            target_pose["x"] = WAYPOINTS[current_waypoint_index][0]
            target_pose["y"] = WAYPOINTS[current_waypoint_index][1] # Using 'y' for consistency with target_pose dict
            print(f"[{robot.getTime():.2f}] IDLE: New Target: x={target_pose['x']:.2f}, y={target_pose['y']:.2f} (Waypoint {current_waypoint_index})")
            robot_state = "TURNING_TO_TARGET"
        else:
            if WAYPOINTS: # Only print "all waypoints reached" if there were waypoints
                print(f"[{robot.getTime():.2f}] All A* waypoints reached.")
            robot_state = "FINISHED"

    elif robot_state == "TURNING_TO_TARGET":
        dx = target_pose["x"] - current_pose["x"]
        dy = target_pose["y"] - current_pose["y"] # Using target_pose["y"] (our floor Y)
        desired_theta = math.atan2(dy, dx)

        angle_error = desired_theta - current_pose["theta"]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > ANGLE_THRESHOLD:
            if angle_error > 0: left_motor.setVelocity(-TURN_SPEED); right_motor.setVelocity(TURN_SPEED)
            else: left_motor.setVelocity(TURN_SPEED); right_motor.setVelocity(-TURN_SPEED)
        else:
            left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)
            print(f"[{robot.getTime():.2f}] Aligned. Cur_θ={math.degrees(current_pose['theta']):.1f}°, Target_θ={math.degrees(desired_theta):.1f}°")
            robot_state = "MOVING_TO_TARGET"

    elif robot_state == "MOVING_TO_TARGET":
        dx = target_pose["x"] - current_pose["x"]
        dy = target_pose["y"] - current_pose["y"] # Using target_pose["y"] (our floor Y)
        distance = math.sqrt(dx**2 + dy**2)

        desired_theta_while_moving = math.atan2(dy, dx)
        angle_error_while_moving = desired_theta_while_moving - current_pose["theta"]
        angle_error_while_moving = math.atan2(math.sin(angle_error_while_moving), math.cos(angle_error_while_moving))

        if abs(angle_error_while_moving) > ANGLE_THRESHOLD * 1.5: # Allow slightly more deviation while moving before re-orienting
            # print(f"[{robot.getTime():.2f}] MOVING: Drifting (err {math.degrees(angle_error_while_moving):.1f}°). Re-orient.")
            robot_state = "TURNING_TO_TARGET" # Go back to turn
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
        elif distance > DISTANCE_THRESHOLD:
            # Optionally, add P-controller for slight steering correction while moving
            # steering_correction = KP_ANGLE_MOVE * angle_error_while_moving
            # vL = MOVE_SPEED - steering_correction
            # vR = MOVE_SPEED + steering_correction
            # left_motor.setVelocity(vL)
            # right_motor.setVelocity(vR)
            # For now, just move straight after alignment
            left_motor.setVelocity(MOVE_SPEED)
            right_motor.setVelocity(MOVE_SPEED)
        else:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print(f"[{robot.getTime():.2f}] Arrived at Target: x={target_pose['x']:.2f}, y={target_pose['y']:.2f}. Dist: {distance:.3f}m")
            current_waypoint_index += 1
            robot_state = "IDLE"

    elif robot_state == "FINISHED":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        # Periodic idle message if needed
        if loop_count % 200 == 0 : print(f"[{robot.getTime():.2f}] Robot FINISHED. Idling.")

# Cleanup MQTT on exit
cleanup_mqtt()
print(f"[{robot.getTime():.2f}] Controller exiting.")