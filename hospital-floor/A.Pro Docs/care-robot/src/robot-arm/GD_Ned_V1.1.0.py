"""
Simulating a 3DOF Robotic Arm:
This script models the forward and inverse kinematics of a 3-degree-of-freedom robotic arm.
It calculates the end-effector position based on given joint angles and also determines joint angles
for a desired end-effector position using inverse kinematics. Additionally, it employs gradient descent
for optimization to minimize the error between a target and calculated position.

Key functionalities:
- Mapping angles to joint constraints
- Forward and inverse kinematics computations
- Gradient descent optimization for inverse kinematics
- Distance calculation to measure end-effector error
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Dictionary defining joint motion limits and corresponding angles
joint_range_dict = { 'm1': ( 0.8, -0.8, 45, 135 ),   # 'joint_name': (limit_1, limit_2, angle_1, angle_2)
               'm2': ( 1.3, -1.5, 15, 176.57 ),
               'm3': ( 1.5, -1.5, 3.43, 176.57 ),
               'base': ( 2.8, -2.8, 175, -175),
               'twist1': ( 2, -2, -15, 195),
               'twist2': ( 2.5, -2.5, -142, 322)}

# Link lengths of the robotic arm
link_lengths = (24, 210, 230) # l1, l2, l3

def set_degrees(degree, joint, joint_range_dict): # Returns 
	values = joint_range_dict[str(joint)]

	x = round(values[0] - ( (degree-values[2]) * ((values[0] - values[1]) / ((values[3] - values[2])))) , 2)

	# to keep in bounds
	x = min(values[0], x)
	x = max(values[1], x)
	return x


def alpha_to_theta(alpha_list, constraints_radians):
    '''
    Map unbounded parameters alpha to constrained angles theta using sigmoid reparameterization.
    
    Parameters:
        alpha_list (list): List of unbounded parameters [alpha_1, alpha_2, ..., alpha_n].
        constraints_radians (list of tuples): List of (theta_min, theta_max) angles in radians for each joint.
    
    Returns:
        theta_list (list): List of constrained angles [theta_1, theta_2, ..., theta_n] in radians.
    '''
    theta_list = []
    for alpha, (theta_min, theta_max) in zip(alpha_list, constraints_radians):
        sigmoid_alpha = 1 / (1 + math.exp(-alpha))
        theta = theta_min + (theta_max - theta_min) * sigmoid_alpha
        theta_list.append(theta)
    return theta_list


def theta_to_alpha(theta_list, constraints_radians, epsilon=1e-6):
    '''
    Map constrained angles theta to unbounded parameters alpha.
    
    Parameters:
        theta_list (list): List of angles [theta_1, theta_2, ..., theta_n] in radians.
        constraints_radians (list of tuples): List of (theta_min, theta_max) angles in radians for each joint.
        epsilon (float): Small value to prevent division by zero or log of zero.
    
    Returns:
        alpha_list (list): List of unbounded parameters [alpha_1, alpha_2, ..., alpha_n].
    '''
    alpha_list = []
    for theta, (theta_min, theta_max) in zip(theta_list, constraints_radians):
        s = (theta - theta_min) / (theta_max - theta_min)
        s = max(epsilon, min(1 - epsilon, s))
        alpha = math.log(s / (1 - s))
        alpha_list.append(alpha)
    return alpha_list


def position_3DOF(links, thetas): # mm , radians
    '''Calculate the XYZ coordinates of the endpoint of a 3DOF robotic arm.'''
    l1, l2, l3 = links
    q1, q2, q3 = thetas # in radians
    x = ((l2 * math.cos(q2)) + (l3 * math.cos(q2 + q3))) * math.sin(q1)
    y = ((l2 * math.cos(q2)) + (l3 * math.cos(q2 + q3))) * math.cos(q1)
    z = (l2 * math.sin(q2)) + (l3 * math.sin(q2 + q3)) + l1
    # Source: https://www.researchgate.net/publication/353075915_Kinematic_Analysis_for_Trajectory_Planning_of_Open-Source_4-DoF_Robot_Arm
    
    return (x, y, z)


# Cost Function
def distance(current, target):
    '''Calculate the Euclidean distance between two points in 3D space.'''
    return math.sqrt((current[0] - target[0]) ** 2 + (current[1] - target[1]) ** 2 + (current[2] - target[2]) ** 2)


# Adjust angle within 0 to 2π
def adjusted_angle(angle):
    return angle % (math.pi)


def descent(func, params, cost, constraints, n = 100, delta = pow(10, -6), threshold = pow(10, -3), k = 0.01):
    original = cost(func(params))
    descended = params + []
    while True:
        vect = [0 for i in range(len(params))]
        for each in range(len(params)):
            newp = descended + []
            newp[each] += delta
            print('Newp:', newp, cost(func(newp)))
            deriv = (cost(func(newp)) - original)/delta
            vect[each] = (-1 * deriv * k)
            descended[each] += vect[each]
            
            #descended[each] = max(constraints[each][0], min(descended[each], constraints[each][1]))

            original = cost(func(descended))
        if cost(func(descended)) <= threshold:
            return descended
            break

constraints = ( (45, 135),
                (15, 175),
                (4, 175) )

# Convert each angle to radians
constraints_radians = []
for angle_pair in constraints:
    rad_pair = (math.radians(angle_pair[0]), math.radians(angle_pair[1]))
    constraints_radians.append(rad_pair)

# Convert the list back to a tuple
constraints_radians = tuple(constraints_radians)


def inverse_kinematics_3DOF(pos, links, s1, s2):
    l1, l2, l3 = links
    x, y, z = pos
    q1 = math.atan2(y, z)
    r = math.sqrt(x**2 + y**2) * s1
    D = math.sqrt((z - l1)**2 + r**2) * s2
    q3 = math.acos((D**2 - l3**2 - l2**2) / (2 * l2 * l3))
    q2 = math.atan2(r, z - l1) - math.atan2(l2 + (l3 * math.cos(q3)) , l3 * math.sin(q3))

    return q1, q2, q3


target_pos = (100, 100, 50)
angeles_IK = 0.79, -1.35, 2.82

initial_params = (1, 2, 1)
print(constraints_radians)
print(theta_to_alpha(initial_params, constraints_radians))

x_ik, y_ik, z_ik = inverse_kinematics_3DOF(target_pos, link_lengths, 1, 1)
print(f'Angles IK: {x_ik:.2f}, {y_ik:.2f}, {z_ik:.2f}')


test_descent = 1
if test_descent == True:
    for i in range(1):
        # descent(func, params, cost, constraints, n = 100, delta = pow(10, -6), threshold = pow(10, -3), k = 0.01):
        angles = (descent(lambda alpha: position_3DOF(link_lengths, alpha_to_theta(alpha, constraints_radians)),
                        theta_to_alpha(initial_params, constraints_radians),
                        lambda x: distance(x, target_pos),
                        constraints_radians,
                        100,
                        delta = pow(10, -4),
                        threshold = 0.5,
                        k = 0.001))

        angles = alpha_to_theta(angles, constraints_radians)

        #print('Angles:', breh , ' ADJ: ' , adjusted_angle(breh[0]),adjusted_angle(breh[1]),adjusted_angle(breh[2]))
        print(f'Angles GD: {angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f}')
        print(f'Position GD:  {position_3DOF(link_lengths, angles)[0]:.1f}, {position_3DOF(link_lengths, angles)[1]:.1f}, {position_3DOF(link_lengths, angles)[2]:.1f}')
        print(f'Distace: {distance(position_3DOF(link_lengths, angles), target_pos):.2f}')
    #for idc, i in enumerate(breh):
        #print(math.degrees(i), set_degrees(math.degrees(i), ['base','m1','m2'][idc], joint_range_dict))




