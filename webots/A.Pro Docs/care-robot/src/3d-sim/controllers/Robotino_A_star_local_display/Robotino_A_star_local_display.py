"""Robotino_careobot controller."""

from controller import Robot, DistanceSensor, Motor, Keyboard, GPS, Compass, Display, TouchSensor, Emitter, Receiver
import math
import pickle
import numpy as np
from simple_pid import PID
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from scipy.ndimage import distance_transform_edt
import time


WHEEL_RADIUS = 0.063 #meter
ROBOT_RADIUS = 0.2266 #meter
DISTANCE_WHEEL_TO_ROBOT_CENTRE = 0.1826
IR_ANGLES = [90.0, 130.0, 170.0, 210.0, 250.0, 290.0, 330.0, 10.0, 40.0]
IR_12oclock = 0.63 #meters max dist
ROOM_X = 10 # meters
ROOM_Y = 10 # meters
WIDTH = 100
ROWS = 50 # The larger the rows the slower the calculation
GPS_ORIGIN = [4.51800361  ,  -4.4876655]
HOME_POS = [3.10810397  ,  -2.95265777]

RED = 0xff0000
GREEN = 0x64ff00
BLUE = 0x00ff00
YELLOW = 0xffff00
WHITE = 0xffffff
BLACK = 0x000000
PURPLE = 0x3200ff
ORANGE = 0xffa500
GREY = 0xe6e6e6
TURQUOISE = 0x40e0d0


grid_file_address = 'D:\\OneDrive - Ostbayerische Technische Hochschule Amberg-Weiden\\My Projects\\Care_o_bots_Webots\\grid_data.pkl'



# Initialize Devices

# IR Sensors: ir1(12oclock), ir2(11oclock), ... , ir9(1oclock) (clock from top view)
# IR Sensor: 0.31 when nothing , 3.0 when touching
# Bumper Touch Sensor: touch sensor
# Compass: compass
# Gyro: gyro
# Gps: Gps
# IMU: inertial unit
# Accelerometer: accelerometer
# Motors:  wheel0_joint(back), wheel1_joint(rignt), wheel2_joint(left)
# wheel0_joint_sensor
# Emitter: RT_emitter


robot = Robot() # create the Robot instance.
TIME_STEP = int(robot.getBasicTimeStep()) # get the time step of the current world.
keyboard = Keyboard()
keyboard.enable(TIME_STEP)
display = robot.getDevice('display')
bumper = robot.getDevice("touchsensor")
bumper.enable(TIME_STEP)

# IR Sensors Initialize
ir = []
irNames = ['ir1','ir2','ir3','ir4','ir5','ir6','ir7','ir8','ir9',]
for i in range(9):
    ir.append(robot.getDevice(irNames[i]))
    ir[i].enable(TIME_STEP)

# Motor Initialize
Lmotor = robot.getDevice('wheel2_joint')
Rmotor = robot.getDevice('wheel1_joint')
Bmotor = robot.getDevice('wheel0_joint')

Lodo = robot.getDevice('wheel2_joint_sensor').enable(TIME_STEP)
Rodo = robot.getDevice('wheel1_joint_sensor').enable(TIME_STEP)
Bodo = robot.getDevice('wheel0_joint_sensor').enable(TIME_STEP)

Lmotor.setPosition(float('inf'))
Rmotor.setPosition(float('inf'))
Bmotor.setPosition(float('inf'))

Lmotor.setVelocity(0.0)
Rmotor.setVelocity(0.0)
Bmotor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

emitter = robot.getDevice('RT_emitter')
#emitter.enable(TIME_STEP)

receiver = robot.getDevice('RT_receiver')
receiver.enable(TIME_STEP)


class Spot: # ROHAN
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.occupancy = 1

    def get_pos(self):
        return self.row, self.col

    def is_open(self):
        return self.color == GREY

    def is_barrier(self):
        if self.color == BLACK:
            return 1
        return 0

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def is_path(self):
        return self.color == PURPLE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_open(self):
        self.color = GREY

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def occupancy_color(self,value):
        self.color = 100 * value

    def make_path(self):
        self.color = PURPLE

    def draw(self):
        display.setColor(self.color)
        display.fillRectangle(self.x, self.y, self.width, self.width)


# ROHAN
def make_grid(rows, width): # this function creates new "Spots instances" for the grid, spits out an instantiated grid object
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid # each element has an instance of the Spot class

# ROHAN
def load_grid_file(file):
    with open(file, "rb") as file:
        loaded_grid = pickle.load(file)
        print("LOADED: ")
    return loaded_grid

# ROHAN
def save_grid_file(objects, file):
    with open(file, "wb") as file:
        pickle.dump(objects,file)
        print("SAVED: ")

def draw(grid, rows, width):
    for row in grid:
        for spot in row:
            spot.draw() # actually draws the rectangles stored in thr grid.(whatever their atribs are)

# ROHAN
def show_grid_window(win, grid, width):
    draw(win, grid, ROWS, width)
    edit_path(path,grid)

# ROHAN
def start_node(grid):
    for rows in grid: # Finding the Start Node
        for obj in rows:
            if obj.is_start():
                node = [obj.row,obj.col]
    return node

# ROHAN
def new_start_node(grid, gps_pxl):
    for rows in grid: # Finding the Start Node
        for obj in rows:
            if obj.is_start():
                obj.color = WHITE
    
    r, c = pxl_to_RC(gps_pxl)
    grid[r][c].make_start()
    return r, c

# ROHAN
def clear_start_end_path(grid):
    for rows in grid: # Finding the Start Node
        for obj in rows:
            if obj.is_start():
                obj.color = WHITE
            if obj.is_end():
                obj.color = WHITE
            if obj.is_open():
                obj.color = WHITE
            if obj.is_path():
                obj.color = WHITE         

# ROHAN
def end_node(grid):
    for rows in grid: # Finding the End Node
        for obj in rows:
            if obj.is_end():
                node = [obj.row,obj.col]
    return node

# JANVI
def compute_cost_map(grid): # JANVI
    #Making Boundary wall (TEMPORARY) (TEMPORARY) (TEMPORARY)
    for i in range(len(grid)):
        grid[0][i].make_barrier()  # Top row
        grid[-1][i].make_barrier()  # Bottom row
        grid[i][0].make_barrier()  # Left column
        grid[i][-1].make_barrier()  # Right column

    grid_matrix = [[row[i].is_barrier() for row in grid] for i in range(len(grid[0]))] # convert grid to boolean or numeric matrix

    # Euclidean distance transform
    occupancy_grid = np.array(grid_matrix)
    cost_map = distance_transform_edt(occupancy_grid == 0)

    # Normalize the cost map
    max_cost = np.max(cost_map)
    cost_map[cost_map == 1] = 0
    if max_cost > 0:
        cost_map = cost_map / max_cost

    cost_map = 1 - cost_map
 
    cost_map[cost_map == 1] = 0
    return cost_map.tolist()

# JANVI
def set_occupancy_color_in_grid(cost_map,grid):
    cm = np.array(cost_map)
    #grid.occupancy_color(i*255)
    for rows in grid: # Finding the End Node
        for obj in rows:
            if obj.color == WHITE:
                obj.occupancy_color((cm[obj.col,obj.row]*150))

# JANVI
def algorithm(grid, start, end):
    #grid_matrix = [[row[i].is_empty() for row in grid] for i in range(len(grid[0]))] # convert grid to boolean or numeric matrix
    #print(grid_matrix)
    grid = Grid(matrix=grid)
    start_node_ = grid.node(start[0],start[1])
    end_node_ = grid.node(end[0],end[1])
    finder = AStarFinder(diagonal_movement = DiagonalMovement.always)
    path_obj , runs = finder.find_path(start_node_, end_node_, grid)

    path = [[obj.x,obj.y] for obj in path_obj] # NOW YOU HAVE THE PATH!!!!
    #print(path)
    return path

# ROHAN
def edit_path(path, grid):
    for i in path:
        row = i[0]
        col = i[1]
        if not grid[row][col].is_start() and not grid[row][col].is_end():
            grid[row][col].make_path()

# ROHAN
def clear_path(grid):
    for rows in grid:
        for obj in rows:
            if obj.is_path(): obj.make_open()

# JANVI
def path_to_pxl(path):
    path_pxl = []
    for i in path:
        path_pxl.append([i[1]*(WIDTH/ROWS),i[0]*(WIDTH/ROWS),])

    return path_pxl

# ROHAN
def pure_pursuit(path, current, lookout_dist):
    if not path:
        raise ValueError("Path is empty. Cannot calculate look-ahead point.")

    distances = [math.sqrt((point[0] - current[0])**2 + (point[1] - current[1])**2) for point in path]
    
    min_value = min(distances)
    min_index = distances.index(min_value)
    
    if min_value > lookout_dist:
        f = 0.3
        #var_lookout_dist = math.ceil(lookout_dist + (min_value**f)) - 1
        var_lookout_dist = lookout_dist
    else:
        var_lookout_dist = lookout_dist

    look_ahead_index = min_index + var_lookout_dist
    #print(look_ahead_index, "   ", len(path))


    if look_ahead_index >= len(path):
        return path[-1]  # Return the last point if out of bounds

    return path[look_ahead_index]

# JANVI
def pxl_to_RC(pxl):
    r, c = round(pxl[1] / (WIDTH / ROWS)) , round(pxl[0] / (WIDTH / ROWS))
    if r >= ROWS: r = ROWS -1
    if c >= ROWS: c = ROWS -1
    return r, c

# ROHAN
def update_map_from_IR(grid, gps_pxl, IR_polygon, threshold):
    for values in IR_polygon:
        values_rc = pxl_to_RC(values)
        d = math.sqrt( (values[0] - gps_pxl[0])**2 + (values[1] - gps_pxl[1])**2 )
        if d <= threshold:
            grid[values_rc[0]][values_rc[1]].make_barrier()
            return True

# JANVI  
def gps_to_pxl(gpsx,gpsy):
    X_o = abs(gpsx - GPS_ORIGIN[0]) # aligning origins
    Y_o = abs(gpsy - GPS_ORIGIN[1])

    gps_to_pxl_X = X_o * (WIDTH/ROOM_X) / 0.90436
    gps_to_pxl_Y = Y_o * (WIDTH/ROOM_X) / 0.90436

    return gps_to_pxl_X, gps_to_pxl_Y

# JANVI
def meter_to_pxl(value):
    return (value*(WIDTH/ROOM_X))
    IR

IR_position_in_pxl = meter_to_pxl(ROBOT_RADIUS)
IR_12oclock_in_pxl = meter_to_pxl(IR_12oclock)
IR_to_pxl_factor = (IR_12oclock_in_pxl - IR_position_in_pxl) / (3.0 - 0.31)

# ROHAN
def ir_read_pxl(ir_list):
    irValues = []
    for i in range(len(ir_list)):
        irValues.append((IR_12oclock_in_pxl - ( (IR_12oclock_in_pxl - IR_position_in_pxl) * ( ir_list[i].getValue() - 0.31) / (3 - 0.31) )))
    return irValues

# ROHAN
def calculate_centroid(polygon, gps_pxl):
    area = 0
    Cx = 0
    Cy = 0
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]  # Wrap to first vertex for the last edge
        cross = x1 * y2 - x2 * y1  # Cross product
        area += cross
        Cx += (x1 + x2) * cross
        Cy += (y1 + y2) * cross

    area *= 0.5
    
    Cx /= (6 * area)
    Cy /= (6 * area)

    dist = math.sqrt( (Cx - gps_pxl[0])**2 + (Cy - gps_pxl[1])**2 )
    
    return Cx, Cy, dist

# ROHAN
def obstacle_polygon(ir_values_in_pxl, angles, gps_pxl, bearing_deg):
    # values invertor
    polygon = []
    j = 0
    for i in ir_values_in_pxl:
        polygon.append([(gps_pxl[0] + i * math.cos(math.radians(bearing_deg - 90 + angles[j]))) , (gps_pxl[1] + i * math.sin(math.radians(bearing_deg - 90 + angles[j]))) ])      
        j = j + 1
    return polygon

'''
def target_to_centroid_resultant(target_pxl, centroid_pxl):
    xr = (centroid_pxl[0] + target_pxl[0]) / 2
    yr = (centroid_pxl[1] + target_pxl[1]) / 2
    
    return xr, yr
'''
# ANJHITA
def bearing(): # ANJHITA
    compX , compY , compZ = compass.getValues()
    theta = math.atan2(compX , compY) 
    bearing = math.degrees(theta)
    if bearing < 0.0:
        pass
        #bearing = bearing + 360
    #bearing = 360 + bearing
    #print(bearing)
    return bearing

# ANJHITA
def translate(deg, omega, Speed=1, distance='inf'): # ANJHITA
    rad = math.radians(deg+30-90) 
    r = WHEEL_RADIUS
    d = DISTANCE_WHEEL_TO_ROBOT_CENTRE
    omega *= DISTANCE_WHEEL_TO_ROBOT_CENTRE / WHEEL_RADIUS
    
    Vx = Speed * math.cos(rad)
    Vy = Speed * math.sin(rad)
    
    A = np.array([[1,-0.5,-0.5],[0,-math.sqrt(3)/2, math.sqrt(3)/2],[1,1,1]])
    B = np.array([Vx,Vy,omega])
    C = np.linalg.solve(A,B)
    
    Lmotor.setVelocity(C[0])
    Rmotor.setVelocity(C[1])
    Bmotor.setVelocity(C[2])

def rotate(deg, bearing):
    pid_rotate = PID(0.01, 0.01, 0.0001, setpoint = 0)
    return pid_rotate(deg - bearing)

# ANJHITA
def calc_degree(p1,p2): # ANJHITA
    try:
        delta_x = p2[0] - p1[0]
        delta_y = p2[1] - p1[1]
        distance_to_target = math.sqrt(delta_x**2 + delta_y**2)
        theta =  math.atan2(delta_y, delta_x)  # atan2 automatically handles signs
        if theta < 0.0:
            #theta = theta + (2 * math.pi)
            pass
        degree = math.degrees(theta)
        degree =  degree  - bearing() + 90
        
        #print(degree)
    except ZeroDivisionError:
        pass
    return degree

# ANJHITA
def calc_omega(degree, P, I, D): # ANJHITA
    pid_omega = PID(P, I, D, setpoint = 0)
    degree = degree - 90
    if degree > 180: degree = degree - 360
    if degree < -180: degree = 360 - degree

    return pid_omega(degree)

# ANJHITA
def calc_speed(p1,p2,speed,f,threshold): # ANJHITA
    distance_to_target = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    #print(distance_to_target)
    if distance_to_target < threshold:
            speed = min(speed, (distance_to_target / f))  # Scaled speed
    return speed

def calc_speed_PID(p1, p2, max_speed):
    distance_to_target = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    pid_speed = PID(1.5, 0.8, 0.001 , setpoint = 0)
    speed = abs(pid_speed(distance_to_target))

    return min(speed, max_speed)

def dist_calc(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


stuck = [False, 0.0, (0,0)]  # stuck state, last time, last gps position

# ANJHITA
def is_stuck(threshold,stuck,gps_pxl):
    current_time = time.time()
    diff = current_time - stuck[1]
    if diff >= 2:
        stuck[1] = current_time
        last_gps_pxl = stuck[2]
        dist =  math.sqrt( (last_gps_pxl[1] - gps_pxl[1])**2 + (last_gps_pxl[0] - gps_pxl[0])**2 )
        #print(dist)
        stuck[2] = gps_pxl
        if dist < threshold:
            stuck[0] = True
        else:
            stuck[0] = False
        return stuck
    return stuck

def rp(v):
    return round(v, 1)



def read_receiver():
    data = ''
    while receiver.getQueueLength() > 0 and robot.step(TIME_STEP) != -1:
        data = receiver.getString()
        receiver.nextPacket()
    return data

# Wait for Signal:
def get_serial_data():
    signal = read_receiver()
    if signal == "Robotino Reached":
        print("RECEIVED: " + signal )
    return signal









#____________________________________________________________________________________
# MAIN loop:

grid = make_grid(ROWS, WIDTH)
grid[5][5].make_start()
grid[38][9].make_end()
#run = True
while robot.step(TIME_STEP) != -1:
    grid_loaded = False
    run = False
    calc_path = True
    stuck_mode = False
    path_follow_mode = True
    explore_mode = False
    navigation = True
    reached = False
    ir_threshold = max(ir_read_pxl(ir))-0.015

    print("Robotino Controller\n==================\n")
    print("[1] Load Grid File")
    print("[2] Display Grid")
    print("[3] Make New Grid")
    print("[4] Explore Grid")
    print("[Save] Save Current Grid")
    print('[R] Run')

    while not run and robot.step(TIME_STEP) != -1: # Menu Selection Loop
        key=keyboard.getKey()

        if key==ord('1'):
            grid = load_grid_file(grid_file_address)
            #print(grid)
            grid_loaded = True
            run = False
            time.sleep(0.2)

        elif key==ord('2'):
            time.sleep(0.2)

        elif key==ord('3'):
            if not grid_loaded: grid = make_grid(ROWS, WIDTH)
            grid_loaded = True
            print("NEW GRID CREATED")
            run = False
            time.sleep(0.2)

        elif key==ord('4'):
            explore_mode = True
            print("EXPLORE MODE - True")
            time.sleep(0.2)

        elif key==ord('5'):
            explore_mode = True
            print("SENDING")
            emitter.send("Robotino Reached")
            time.sleep(0.2)

        elif key==ord('S'):
            save_grid_file(grid,grid_file_address)
            time.sleep(0.2)

        elif key==ord('R') and grid_loaded:
            grid[5][5].make_start()
            grid[40][8].make_end()
            run = True
            #time.sleep(5)


    # ANJHITA
    # Run Loop, the Navigation starts here.
    while run and robot.step(TIME_STEP) != -1: 

        # Map Calculation
        if calc_path == True and navigation == True:
            cost_map = compute_cost_map(grid)
            path = algorithm(cost_map, start_node(grid), end_node(grid))
            clear_path(grid)
            edit_path(path,grid)
            #set_occupancy_color_in_grid(cost_map,grid)
            calc_path = False

        #show_grid_window(WIN, grid, WIDTH)
        draw(grid, ROWS, WIDTH)

            
        gpsX , gpsY , gpsZ = gps.getValues()
        gpsX = round(gpsX * 100000 , 8)
        gpsY = round(gpsY * 100000 , 8)
        #print("       ",gpsX, " , ",gpsY)
        gps_pxl = gps_to_pxl(gpsX,gpsY)
        look_ahead = pure_pursuit(path_to_pxl(path), gps_pxl, 1)
        IR_polygon = obstacle_polygon(ir_read_pxl(ir), IR_ANGLES, gps_pxl, bearing())
        cx , cy , centroid_dist = calculate_centroid(IR_polygon, gps_pxl)

        display.setColor(RED)
        display.fillOval(gps_pxl[1], gps_pxl[0], 2,2)




        

        # MODES
        # -----------------------------------------------------------------
        if explore_mode == True:
            if update_map_from_IR(grid, gps_pxl, IR_polygon, ir_threshold):
                calc_path = True

        if look_ahead == path_to_pxl(path)[-1] and path_follow_mode == True:
            #save_grid_file(grid,grid_file_address)
            print("Path Finding Complete")
            path_follow_mode = False
            stuck_mode = False
            explore_mode = False
            #run = False
            navigation = False
            clear_path(grid)
            clear_start_end_path(grid)
            translate(0,0,0)
            #time.sleep(5)

        if path_follow_mode == True:
            degree = calc_degree(gps_pxl, look_ahead)
            speed = calc_speed(gps_pxl, look_ahead, 10, 0.01, 5)
            omega = calc_omega(degree, 0.08, 0.01, 0.0001)
            #calc_omega(degree, bearing())
            if centroid_dist > 0.1:
                avoidance_degree =  calc_degree(gps_pxl, (cx , cy))
                if explore_mode == True:
                    update_map_from_IR(grid, gps_pxl, IR_polygon, ir_threshold)
                #translate(avoidance_degree,0, 20)
            else:
                translate(degree, omega, speed)
                pass
               
            stuck = is_stuck(1,stuck,gps_pxl)

            if stuck[0] == True:
                print("Stuck!")
                path_follow_mode = False
                stuck_mode = True
                if explore_mode == True:
                    start = new_start_node(grid,gps_pxl)
                translate(0,0,0)

        if bumper.getValue() == 1:
            stuck_mode = True
            start = new_start_node(grid,gps_pxl)
            print('Bump!')

        if stuck_mode == True:
            stuck[0] = False
            stuck = is_stuck(3,stuck,gps_pxl)

            t1 = time.time()
            while time.time() - t1 < 0.5  and robot.step(TIME_STEP) != -1:
                gpsX , gpsY , gpsZ = gps.getValues()
                gpsX = round(gpsX * 100000 , 8)
                gpsY = round(gpsY * 100000 , 8)
                gps_pxl = gps_to_pxl(gpsX,gpsY)
                look_ahead = pure_pursuit(path_to_pxl(path), gps_pxl, 2)
                IR_polygon = obstacle_polygon(ir_read_pxl(ir), IR_ANGLES, gps_pxl, bearing())
                degree = calc_degree(gps_pxl, look_ahead)
                translate(0,5,0)
                stuck[0] = False
                update_map_from_IR(grid, gps_pxl, IR_polygon, ir_threshold)
                avoidance_degree =  calc_degree(gps_pxl, (cx , cy))
                translate(avoidance_degree,5, 20)

            calc_path = True
            stuck_mode = False
            path_follow_mode = True
            print("Explored")
            translate(degree,0,20)

        #  till here ANJHITA

        # Final Navigation
        if navigation == False and reached == False:
            final_point = (15, 88)
            current_bearing = bearing()
            degree = calc_degree(gps_pxl, final_point)
            speed = calc_speed_PID(gps_pxl, final_point, 5)
            omega = calc_omega(current_bearing - 180 , 0.01, 0, 1)

            if dist_calc(gps_pxl, final_point) > 0.5:
                translate(degree, omega, speed)

            if current_bearing > 89 and current_bearing < 91:
                translate(degree, 0, speed) 

            if dist_calc(gps_pxl, final_point) <= 0.5 and current_bearing > 89 and current_bearing < 91:
                translate(0, 0, 0) # STOP
                print("Reached")
                reached = True
                emitter.send("Robotino Reached")


        # Transmit signal
        if reached == True:
            if get_serial_data() == 'Bottle Picked':
                grid = make_grid(ROWS, WIDTH)
                grid[5][5].make_end()
                grid[40][9].make_start()
                run = True
                calc_path = True
                stuck_mode = False
                path_follow_mode = True
                explore_mode = True
                navigation = True
                reached = False
                grid = load_grid_file(grid_file_address)
                grid_loaded = True
            
            pass


print("DONE")



