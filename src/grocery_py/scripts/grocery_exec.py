#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from grocery_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

Q = None

# the left-top position of the shelf
initial_shelf_pos = (-0.965662, -0.185334, 0.524682) 

# distance between each cell of the shelf
cell_dist = 0.11

# size of the shelf (n by n)
shelf_size = 5


"""
Calculate a shelf position 2d array given the initial_shelf_pos
"""
def calculate_shelf_pos(initial_shelf_pos):
    shelf = np.empty([shelf_size, shelf_size], dtype=tuple)
    for i in range(shelf_size):
	for j in range(shelf_size):
	    shelf[i , j] = (initial_shelf_pos[0],
			    initial_shelf_pos[1] + 0.11 * j, 
			    initial_shelf_pos[2] - 0.11 * i)
    # print(shelf)
    return shelf 
					

def gripper_callback(msg):
    global gripper_
    gripper_ = msg.DIGIN


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    move_arm(pub_cmd, loop_rate, Q[start_height][start_loc], 4.0, 4.0)
    rospy.sleep(0.5)
    gripper(pub_cmd, loop_rate, suction_on)
    rospy.sleep(0.5)

    # if no block exit
    if (gripper_ == 0):
        move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
        gripper(pub_cmd, loop_rate, suction_off)
        print("NO BLOCK FOUND. EXITING... ")
        sys.exit()

    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)
    rospy.sleep(0.5)
    move_arm(pub_cmd, loop_rate, Q[end_height][end_loc], 4.0, 4.0)
    rospy.sleep(0.5)
    gripper(pub_cmd, loop_rate, suction_off)
    rospy.sleep(0.5)
    move_arm(pub_cmd, loop_rate, home, 4.0, 4.0)

    error=0

    return error


############### Your Code End Here ###############


########## Lab 4 Inverse Kinematic Function ##########
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):


    # the distance between the end effector and the wrist
    d = 0.0535

    # yaw in radians
    yaw_WgripRadian = (np.pi * yaw_WgripDegree) / 180

    #=================== 1) Find x_cen, y_cen, z_cen ======================#
    x_cen = xWgrip - d * np.cos(yaw_WgripRadian)
    y_cen = yWgrip - d * np.sin(yaw_WgripRadian)
    z_cen = zWgrip

    #print(x_cen, y_cen, z_cen)

    #========================= 2) Find theta1 ==============================#

    # NOTE: this is not the L1 and L2 as the robot links. Just some arbitrary names we gave


    # L2
    L2 = 0.11

    # d1
    d1 = x_cen + 0.15

    # L1 
    L1 = y_cen - 0.15

    # r
    r1 = np.sqrt(d1**2 + L1**2)

    #print(r1)

    # theta_a_radian
    # NOTE: use arccos instead of arcsin
    theta_a_radian = np.arccos(L1 / r1)

    # theta_c_radian
    theta_c_radian = np.arcsin(L2 / r1)

    theta1_radian = np.pi / 2 - theta_a_radian - theta_c_radian


    #============================== 3) Find theta6 =========================#
    
    theta6_radian = np.pi - yaw_WgripRadian - (np.pi / 2 - theta1_radian)



    #==================== 4) Find x_3end, y_3end, and z_3end ===============# 

    # top view distance from green point to endpoint

    

    d_ge_top = r1 * np.cos(theta_c_radian) - 0.083
    
    x_3end = d_ge_top * np.cos(theta1_radian) - 0.15 

    y_3end = d_ge_top * np.sin(theta1_radian) + 0.15

    z_3end = z_cen + 0.141
        
    #==================== 5) Find theta2, theta3, and theta4 ===============#   	

    # L3 
    L3 = 0.244 

    # L5
    L5 = 0.213

    # x1
    x1 = -0.15

    # y1
    y1 = 0.15

    # z1
    z1 = 0.162

    # side view distance from green point to endpoint
    d_ge_side = np.sqrt((x_3end - x1)**2 + (z_3end - z1)**2 + (y_3end - y1)**2)

    theta3_radian = np.pi - np.arccos((L3**2 + L5**2 - d_ge_side**2) / (2 * L3 * L5))


    phi_1 = np.arcsin((z_3end - z1) / (d_ge_side)) 
    # print(d_ge_side)

    #print((L3**2 + d_ge_side**2 - L5**2) / (2 * L3 * d_ge_side))
    phi_2 = np.arccos((L3**2 + d_ge_side**2 - L5**2) / (2 * L3 * d_ge_side))


    theta2_radian = -phi_1 - phi_2

    theta4_radian = - theta2_radian - theta3_radian



    # theta1_radian = 0.0
    # theta2_radian = 0.0
    # theta3_radian = 0.0
    # theta4_radian = 0.0
    theta5_radian = -np.pi/2
    # theta6_radian = 0.0

    # print("theta1: ", theta1_radian)
    # print("theta2: ", theta2_radian)
    # print("theta2: ", theta2_radian)
    # print("theta3: ", theta3_radian)
    # print("theta4: ", theta4_radian)
    # print("theta5: ", theta5_radian)
    # print("theta6: ", theta6_radian)
    thetaArray = [theta1_radian + np.pi, theta2_radian, theta3_radian, theta4_radian - (0.5*np.pi), theta5_radian, theta6_radian] #Adjusted joint angles to match robot definitions

    return thetaArray

#################### End of Lab 4 IK Function ######################


################### Shelf Inverse Kinematic Function ##################
def shelf_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	#all angles are in radians unless otherwise stated
	#list the link lengths of the robot
	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.083
	L7 = 0.083
	L8 = 0.082
	L9 = 0.0535
	L10 = 0.059

	#initialize variable that keeps track of which quadrant the end-effector is in
	quadrant = 1

	#check which quadrant the end-effector is in
	if xWgrip >= -0.15:
		quadrant = 1
	else:
		quadrant = 2

	#adjust the position to be based off of the green point
	if quadrant == 1:
		x_grip = xWgrip + 0.15
	if quadrant == 2:
		x_grip = abs(abs(xWgrip) - 0.3)
		xWgrip = x_grip - 0.15
	y_grip = yWgrip - 0.15
	
	#finding theta1
	theta_a = np.arctan(y_grip / x_grip)

	r0 = np.sqrt((x_grip**2) + (y_grip**2))
	d0 = L8 + L10
	d1 = np.sqrt((r0**2) + (d0**2) - (2 * r0 * d0 * np.cos(theta_a)))
	theta_b = np.arccos(((d1**2) + (r0**2) - (d0**2))/(2 * d1 * r0))

	d2 = L2 - L4 + L6
	theta_c = np.arcsin(d2 / d1)

	theta1 = theta_a + theta_b - theta_c

	#finding theta5
	theta5 = -theta1 - (np.pi/2)

	#finding the centerpoint (the ornage point) 
	x_cen = xWgrip - d0
	y_cen = yWgrip
	z_cen = zWgrip

	#finding the endpoint (the yellow point)
	d3 = L7 + L9

	x3_end = x_cen + (d2 * np.sin(theta1))
	y3_end = y_cen - (d2 * np.cos(theta1))
	z3_end = z_cen - d3

	#defining the green point
	x1 = -0.15
	y1 = 0.15
	z1 = L1 + 0.01

	#finding theta3
	d4 = np.sqrt(((x3_end - x1)**2) + ((y3_end - y1)**2) + ((z3_end - z1)**2))
	theta3 = np.pi - np.arccos(((L3**2) + (L5**2) - (d4**2)) / (2 * L3 * L5))

	#finding theta2 
	phi2 = np.arccos(((L3**2) + (d4**2) - (L5**2)) / (2 * L3 * d4))
	phi1 = np.arcsin((z3_end - z1) / d4)
	theta2 = -phi1 - phi2

	#finding theta4
	theta4 = -theta2 - theta3 - (np.pi / 2)

	#constraining theta6
	theta6 = np.pi / 2

	#adjust theta1 and theta5 if they are in quadrant II
	if quadrant == 2:
		theta1 = np.pi - (2 * theta_c) - theta1
		theta5 = (np.pi / 2) - theta1


	#modify theta values to match robot value definitions IF not using lab_fk to return the value
	# Initialize the return_value
	return_value = [None, None, None, None, None, None]
	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	# ==============================================================#
	return return_value


def main():

    calculate_shelf_pos(initial_shelf_pos)

    global home
    global Q
    global SPIN_RATE

    # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Definition of our tower

    # 2D layers (top view)

    # Layer (Above blocks)
    # | Q[0][2][1] Q[1][2][1] Q[2][2][1] |   Above third block
    # | Q[0][1][1] Q[1][1][1] Q[2][1][1] |   Above point of second block
    # | Q[0][0][1] Q[1][0][1] Q[2][0][1] |   Above point of bottom block

    # Layer (Gripping blocks)
    # | Q[0][2][0] Q[1][2][0] Q[2][2][0] |   Contact point of third block
    # | Q[0][1][0] Q[1][1][0] Q[2][1][0] |   Contact point of second block
    # | Q[0][0][0] Q[1][0][0] Q[2][0][0] |   Contact point of bottom block

    # First index - From left to right position A, B, C
    # Second index - From "bottom" to "top" position 1, 2, 3
    # Third index - From gripper contact point to "in the air" point

    # How the arm will move (Suggestions)
    # 1. Go to the "above (start) block" position from its base position
    # 2. Drop to the "contact (start) block" position
    # 3. Rise back to the "above (start) block" position
    # 4. Move to the destination "above (end) block" position
    # 5. Drop to the corresponding "contact (end) block" position
    # 6. Rise back to the "above (end) block" position

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    grocery_path = rospack.get_path('grocery_py')
    yamlpath = os.path.join(grocery_path, 'scripts', 'grocery_data.yaml')

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
            if args.simulator.lower() == 'true':
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()
            
        except:
            print("YAML not found")
            sys.exit()

    # Initialize ROS node
    rospy.init_node('grocerynode')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)


    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)	



    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    # input_done = 0
    # loop_count = 0

    # while(not input_done):
    #     input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
    #     print("You entered " + input_string + "\n")

    #     if(int(input_string) == 1):
    #         input_done = 1
    #         loop_count = 1
    #     elif (int(input_string) == 2):
    #         input_done = 1
    #         loop_count = 2
    #     elif (int(input_string) == 3):
    #         input_done = 1
    #         loop_count = 3
    #     elif (int(input_string) == 0):
    #         print("Quitting... ")
    #         sys.exit()
    #     else:
    #         print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0][1], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0][1], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)

	#Initializing the shelf position matrix
    Qi = [0.0, 0.0, 0.0]
    shelfQ = [ [Qi, Qi, Qi], \
			   [Qi, Qi, Qi], \
			   [Qi, Qi, Qi] ]
    for i in range(3):
		for j in range(3):
			shelfQ[i][j] = [-0.6182, 0.175 + (0.11 * i), 0.54 - (0.12 * j)]

    #Initializing the supply matrix
    supplyQ = [ [Qi, Qi, Qi], \
			    [Qi, Qi, Qi], \
			    [Qi, Qi, Qi] ]
    for i in range(3):
		for j in range(3):
			supplyQ[i][j] = [0.4 - (0.1 * j), 0.25 - (0.1 * i), 0.026]

    # while(loop_count > 0): #picks up the red block and places it at on top of the green block

	# 	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	# 	move_arm(pub_command, loop_rate, shelf_invk(shelfQ[0][0][0], shelfQ[0][0][1], shelfQ[0][0][2], 0.0), 4.0, 4.0)

	# 	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	# 	move_arm(pub_command, loop_rate, shelf_invk(shelfQ[0][1][0], shelfQ[0][1][1], shelfQ[0][1][2], 0.0), 4.0, 4.0)
	# 	move_arm(pub_command, loop_rate, home, 4.0, 4.0)
	# 	move_arm(pub_command, loop_rate, shelf_invk(shelfQ[0][2][0], shelfQ[0][2][1], shelfQ[0][2][2], 0.0), 4.0, 4.0)

	# 	loop_count = 0

    for i in range(3):
		for j in range (3):
			move_arm(pub_command, loop_rate, home, 4.0, 4.0)
			move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0], shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)
			for k in range (1,11):
				move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0] - (0.01 * k), shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)
			gripper(pub_command, loop_rate, suction_on)
			time.sleep(0.5)
			for k in range (1,11):
				move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0] + (0.01 * k) - 0.1, shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)
			if gripper_ == 0:
				#move to the supply, grab it
				gripper(pub_command, loop_rate, suction_off)
				move_arm(pub_command, loop_rate, home, 4.0, 4.0)
				move_arm(pub_command, loop_rate, lab_invk(supplyQ[i][j][0], supplyQ[i][j][1], supplyQ[i][j][2]+ 0.1, 0.0), 4.0, 4.0)
				for k in range (1,11):
					move_arm(pub_command, loop_rate, lab_invk(supplyQ[i][j][0], supplyQ[i][j][1], supplyQ[i][j][2] + 0.1 - (0.01 * k), 0.0), 4.0, 4.0)
				gripper(pub_command, loop_rate, suction_on)
				for k in range (1,11):
					move_arm(pub_command, loop_rate, lab_invk(supplyQ[i][j][0], supplyQ[i][j][1], supplyQ[i][j][2] + (0.01 * k), 0.0), 4.0, 4.0)
				#move to the shelf, drop it
				move_arm(pub_command, loop_rate, home, 4.0, 4.0)
				move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0], shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)
				for k in range (1,11):
					move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0] - (0.01 * k), shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)
				gripper(pub_command, loop_rate, suction_off)
				for k in range (1,11):
					move_arm(pub_command, loop_rate, shelf_invk(shelfQ[i][j][0] + (0.01 * k) - 0.1, shelfQ[i][j][1], shelfQ[i][j][2], 0.0), 4.0, 4.0)







    ############### Your Code End Here ###############

    



if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass

