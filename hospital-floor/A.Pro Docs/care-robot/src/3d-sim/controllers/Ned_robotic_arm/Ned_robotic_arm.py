from controller import Robot, Receiver, Keyboard, Emitter
import time
import math
import copy

print("Ned is Running")

robot = Robot() # create the Robot instance.
TIME_STEP = int(robot.getBasicTimeStep()) # get the time step of the current world.
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

emitter = robot.getDevice('Ned_emitter')

receiver = robot.getDevice('Ned_receiver')
receiver.enable(TIME_STEP)

# Init the motors - the Ned robot is a 6-axis robot arm
# You can find the name of the rotationalMotors is the device parameters of each HingeJoints
base = robot.getDevice('joint_1')
m1 = robot.getDevice('joint_2')
m2 = robot.getDevice('joint_3')
twist1 = robot.getDevice('joint_4')
m3 = robot.getDevice('joint_5')
twist2 = robot.getDevice('joint_6')
gripper = robot.getDevice('gripper::left')

# Set the motor velocity
# First we make sure that every joints are at their initial positions
m1.setPosition(0)  # 0.8 to -0.8,  0 = 45deg, 0.8 = 0deg, -0.8=90deg
m2.setPosition(0) # 1.3 to -1.5,  1.3 = 15deg, 0 = 90deg, -1.5 = 176.57deg
m3.setPosition(0) # 1.5 to -1.5, 1.5 = -86.57deg, 0 = 0deg, -1.5 = 86.57deg
base.setPosition(1.57)   # 2.8 to -2.8, 0 = 90deg , 1.57 = 0deg , -1.57 = 180deg
twist1.setPosition(0) # 2 to -2, 2 = 105deg, 0 = 0deg, -2 = -105deg
twist2.setPosition(0) # 2.5 to -2.5, 2.5 = 232.5deg, 0 = 0deg, -232.5deg
gripper.setPosition(0) # 0.01 = Open , -0.01 = Close 

# Set the motors speed. Here we set it to 1 radian/second
m1.setVelocity(0.5)
m2.setVelocity(0.5)
m3.setVelocity(0.5)
base.setVelocity(0.5)
twist1.setVelocity(0.5)
twist2.setVelocity(0.5)
gripper.setVelocity(0.5)


range_dict = { "m1": ( 0.8, -0.8, 45, 135 ),   # "joint_name": (limit_1, limit_2, angle_1, angle_2)
			   "m2": ( 1.3, -1.5, 15, 176.57 ),
			   "m3": ( 1.5, -1.5, 3.43, 176.57 ),
			   "base": ( 2.8, -2.8, 175, -175),
			   "twist1": ( 2, -2, -15, 195),
			   "twist2": ( 2.5, -2.5, -142, 322),
			   "gripper": ( 0.01 , -0.01 , 0, 1)}

def set_degrees(degree, joint, name, range_dict):
	values = range_dict[str(name)]
	x = round(values[0] - ( (degree-values[2]) * ((values[0] - values[1]) / ((values[3] - values[2])))) , 2)

	# to keep in bounds
	x = min(values[0], x)
	x = max(values[1], x)

	joint.setPosition(x)

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

# steps = [[115, 74, 85, 66, 94, 0], [115, 74, 85, 66, 94, 0], [115, 74, 85, 66, 94, 0], [115, 74, 85, 66, 94, 0], [115, 74, 85, 66, 94, 0]]

record = False
run = False
coord_list = [99, 107, 66, 86, 90, -2]
save_coords = [[99, 107, 66, 86, 90, -2], [114, 107, 66, 86, 90, -2], [115, 90, 62, 63, 90, -2], [115, 76, 76, 63, 90, -2], [115, 77, 84, 72, 90, 2], [115, 77, 89, 72, 90, 2], [115, 81, 96, 80, 90, 2], [115, 126, 80, 88, 90, 2], [8, 65, 103, 73, 90, 2], [18, 65, 103, 73, 44, 2], [9, 46, 101, 84, -25, 2], [9, 43, 101, 84, -25, 2]]
while robot.step(TIME_STEP) != -1:

	#set_degrees(theta1, base, "base", range_dict)
	#set_degrees(theta2, m1, "m1", range_dict)

	#time.sleep(1)
	#print(coord_list)
	a = 1
	if run == False:
		set_degrees(coord_list[0], base, "base", range_dict)
		set_degrees(coord_list[1], m1, "m1", range_dict)
		set_degrees(coord_list[2], m2, "m2", range_dict)
		set_degrees(coord_list[3], m3, "m3", range_dict)
		set_degrees(coord_list[4], twist1, "twist1", range_dict)
		set_degrees(coord_list[5], gripper, "gripper", range_dict)

	if get_serial_data() == 'Robotino Reached':
		run = True

	key = keyboard.getKey()
	if key == ord('P'):
		record = True

	elif key == ord('Q'):
		save_coords.append(coord_list.copy())
		print(save_coords)	 
		time.sleep(0.5)

	elif key == ord('A'):
		coord_list[0] += a
	elif key == ord('Z'):
		coord_list[0] -= a
		

	elif key == ord('S'):
		coord_list[1] += a
	elif key == ord('X'):
		coord_list[1] -= a
		

	elif key == ord('D'):
		coord_list[2] += a
	elif key == ord('C'):
		coord_list[2] -= a
		
	elif key == ord('F'):
		coord_list[3] += a
	elif key == ord('V'):
		coord_list[3] -= a

	elif key == ord('G'):
		coord_list[4] += a
	elif key == ord('B'):
		coord_list[4] -= a

	elif key == ord('H'):
		coord_list[5] += a
	elif key == ord('N'):
		coord_list[5] -= a

	elif key == ord('L'):
		record = False
		run = True

	while robot.step(TIME_STEP) != -1 and run == True:
		for i in save_coords:
			count = 0
			print(i)
			while robot.step(TIME_STEP) != -1 and count < 80:
				set_degrees(i[0], base, "base", range_dict)
				set_degrees(i[1], m1, "m1", range_dict)
				set_degrees(i[2], m2, "m2", range_dict)
				set_degrees(i[3], m3, "m3", range_dict)
				set_degrees(i[4], twist1, "twist1", range_dict)
				set_degrees(i[5], gripper, "gripper", range_dict)
				count += 1
		run = False
		coord_list = [9, 46, 101, 84, -25, 2]
		emitter.send("Bottle Picked")
		




	#time.sleep(0.02)




	





