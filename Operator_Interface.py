# Operator Interface

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado


### sudo apt-get install python-tk


#import Camera_Interface
from Maze_Generation import *
from Maze_Solver import *
#import Image_Processor
#import Coordinate_Converter
from houghLines import *

import copy
import time
from Tkinter import *
import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Empty

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None
maze_dim = 11
maze = []

''' Positive x is away from the sawyer arm forwards (min x is about 0.23)
	Positive y is away from sawyer to the left
	Positive z is up
'''


def rec_mz():
	global maze
	print("Taking picture of maze.")
	#maze = generate_maze(maze_dim, maze_dim)
	# maze = [[1,0,1,1,1,1,1,1,1,1,1],[1,0,0,0,0,0,1,0,0,0,1],[1,0,1,1,1,0,1,0,1,1,1],[1,0,1,0,1,0,0,0,1,0,1],[1,1,1,0,1,0,1,1,1,0,1],[1,0,1,0,0,0,0,0,0,0,1],[1,0,1,1,1,0,1,1,1,0,1],[1,0,0,0,1,0,0,0,1,0,1],[1,0,1,1,1,0,1,0,1,0,1],[1,0,0,0,0,0,1,0,1,0,1],[1,1,1,1,1,1,1,1,1,0,1]]
	# maze = getMaze()
	# if(len(maze)%2 == 0):
	# 	for i in range(len(maze)):
	# 		maze[i].append(1)
	# 	maze.append([1]*len(maze))
	for i in range(len(maze)):
		print(maze[i])
	xy_list = solveMaze(maze, 0, 1, len(maze)-2, len(maze)-1, len(maze)-1)
	move_arm_sol_mz(xy_list)
	print("Solved!")
	

def gen_mz():
	global maze
	print("Generating a maze.")
	maze = generate_maze(maze_dim, maze_dim) # Takes in m,n for maze dimensions
	# maze = [[1,0,1,1,1,1,1,1,1,1,1],[1,0,0,0,0,0,1,0,0,0,1],[1,0,1,1,1,0,1,0,1,1,1],[1,0,1,0,1,0,0,0,1,0,1],[1,1,1,0,1,0,1,1,1,0,1],[1,0,1,0,0,0,0,0,0,0,1],[1,0,1,1,1,0,1,1,1,0,1],[1,0,0,0,1,0,0,0,1,0,1],[1,0,1,1,1,0,1,0,1,0,1],[1,0,0,0,0,0,1,0,1,0,1],[1,1,1,1,1,1,1,1,1,0,1]]
	for i in range(len(maze)):
		print(maze[i])
	move_arm_gen_mz(maze)
	print("Done")

def cal():
	global g_limb, g_position_neutral, g_orientation_hand_down
	start_x = 0.5
	start_y = 0.02
	pen_down = 0.005
	print("Calibrating corners of maze.")
	g_limb.set_joint_position_speed(0.0)
	g_limb.move_to_neutral()
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
	target_pose.position.x = start_x
	target_pose.position.y = start_y
	target_pose.position.z = .015
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(1)
	target_pose.position.z = pen_down
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)

def ogripper():
	print("Openning gripper.")
	gripper = intera_interface.Gripper()
	gripper.open()


def cgripper():
	print("Closing gripper.")
	gripper = intera_interface.Gripper()
	gripper.close()


def move_arm_gen_mz(xy_pos):
	global g_limb, g_position_neutral, g_orientation_hand_down

	scale_factor = 0.00635 * 2
	grid_size = 0.00635 * 1
	half_grid = grid_size/2.0
	start_x = 0.5
	start_y = 0.02
	pen_up = 0.015
	pen_down = 0.005
	stall_time = 1

	g_limb.set_joint_position_speed(0.0)
	g_limb.move_to_neutral()
	time.sleep(stall_time)
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
	target_pose.position.x = start_x
	target_pose.position.y = start_y - half_grid
	target_pose.position.z = pen_up
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)
	'''
	target_pose.position.x = start_x
	target_pose.position.y = start_y - half_grid
	target_pose.position.z = pen_down
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)
	'''
	
	xy_row_len = len(xy_pos)
	xy_col_len = len(xy_pos[0])
	lines = []
	#vertical lines
	for i in range(xy_row_len):
		j = 0
		while(j < xy_col_len-1):
			x_start = i
			y_start = j
			while(xy_pos[i][j] == 1 and j < xy_col_len-1 and xy_pos[i][j+1] == 1):
				j += 1
			if(j-y_start > 1):
				lines.append([x_start, y_start, i, j])
			j += 1

	#horizontal lines
	for j in range(xy_col_len):
		i = 0
		while(i < xy_row_len-1):
			x_start = i
			y_start = j
			#walls
			while(xy_pos[i][j] == 1 and i < xy_row_len-1 and xy_pos[i+1][j] == 1):
				i += 1
			if(i-x_start > 1):
				lines.append([x_start, y_start, i, j])
			i += 1

	# print("")
	# print(lines)
	for n in range(len(lines)):
		# print(lines[n])
		#move to line start
		target_pose.position.x = start_x + lines[n][0] * scale_factor
		target_pose.position.y = start_y + lines[n][1] * scale_factor
		target_pose.position.z = pen_up
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)

		#put pen down
		target_pose.position.x = start_x + lines[n][0] * scale_factor
		target_pose.position.y = start_y + lines[n][1] * scale_factor
		target_pose.position.z = pen_down
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)

		#draw line
		target_pose.position.x = start_x + lines[n][2] * scale_factor
		target_pose.position.y = start_y + lines[n][3] * scale_factor
		target_pose.position.z = pen_down
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)

		#lift pen up after line is drawn
		target_pose.position.x = start_x + lines[n][2] * scale_factor
		target_pose.position.y = start_y + lines[n][3] * scale_factor
		target_pose.position.z = pen_up
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)

	
	'''
	for i in range(xy_row_len):
		for j in range(xy_col_len - 1):
			#movements while pen is down
			if xy_pos[i][j] == 1 and xy_pos[i][j+1] == 1: # Tuples???????????????
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor + half_grid
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				# g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				# time.sleep(stall_time)
			if xy_pos[i][j] == 0 and xy_pos[i][j+1] == 1:
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor - half_grid
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor - half_grid
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor + half_grid
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			# movements while pen is up
			if xy_pos[i][j] == 1 and xy_pos[i][j+1] == 0:
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + j*scale_factor + half_grid
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor + half_grid
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			if xy_pos[i][j] == 0 and xy_pos[i][j+1] == 0: 
				target_pose.position.x = start_x + i*scale_factor
				target_pose.position.y = start_y + (j+1)*scale_factor + half_grid
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			
		target_pose.position.x = start_x + i*scale_factor
		target_pose.position.y = start_y + (xy_col_len+1)*scale_factor + half_grid
		target_pose.position.z = pen_up
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
			return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)
	for j in range(xy_col_len):
		for i in range(xy_row_len - 1):
			if xy_pos[i][j] == 1 and xy_pos[i+1][j] == 1: # Tuples???????????????
				target_pose.position.x = start_x + (i+1)*scale_factor + half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			if xy_pos[i][j] == 0 and xy_pos[i+1][j] == 0: 
				target_pose.position.x = start_x + (i+1)*scale_factor + half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			if xy_pos[i][j] == 0 and xy_pos[i+1][j] == 1:
				target_pose.position.x = start_x + (i+1)*scale_factor - half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + (i+1)*scale_factor - half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + (i+1)*scale_factor + half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_down
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
			if xy_pos[i][j] == 1 and xy_pos[i+1][j] == 0:
				target_pose.position.x = start_x + i*scale_factor + half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
				target_pose.position.x = start_x + (i+1)*scale_factor + half_grid
				target_pose.position.y = start_y + j*scale_factor
				target_pose.position.z = pen_up
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
					return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				time.sleep(stall_time)
		target_pose.position.x = start_x + (xy_row_len+1)*scale_factor + half_grid
		target_pose.position.y = start_y + j*scale_factor
		target_pose.position.z = pen_up
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
			return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)
		'''


def init():
	global g_limb, g_orientation_hand_down, g_position_neutral
	rospy.init_node('sawyer_ik_setup')
	g_limb = intera_interface.Limb('right')


	# This quaternion will have the hand face straight down (ideal for picking tasks)
	g_orientation_hand_down = Quaternion()
	g_orientation_hand_down.x = 0.704238785359
	g_orientation_hand_down.y =0.709956638597
	g_orientation_hand_down.z = -0.00229009932359
	g_orientation_hand_down.w = 0.00201493272073

	# This is the default neutral position for the robot's hand (no guarantee this will move the joints to neutral though)
	g_position_neutral = Point()
	g_position_neutral.x = 0.449559195663
	g_position_neutral.y = 0.16070379419
	g_position_neutral.z = 0.212938808947

def move_arm_sol_mz(xy_pos):
	global g_limb, g_position_neutral, g_orientation_hand_down

	print(xy_pos)

	scale_factor = 0.00635 * 2
	grid_size = 0.00635 * 1
	half_grid = grid_size/2.0
	start_x = 0.5
	start_y = 0.02
	pen_up = 0.015
	pen_down = 0.005
	stall_time = 1

	#move arm to start/entrance of maze
	g_limb.set_joint_position_speed(0.0)
	g_limb.move_to_neutral()
	time.sleep(stall_time)
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
	target_pose.position.x = start_x
	target_pose.position.y = start_y
	target_pose.position.z = pen_up
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)

	'''
	#put pen down
	target_pose.position.x = start_x
	target_pose.position.y = start_y
	target_pose.position.z = pen_down
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)

	#draw find the next line to draw
	i = 0
	while(i < len(xy_pos)-1):
		x_start = xy_pos[i][0]
		y_start = xy_pos[i][1]
		x_end = 0
		y_end = 0
		if(i < len(xy_pos)-1 and xy_pos[i][0] == xy_pos[i+1][0]):
			while(i < len(xy_pos)-1 and xy_pos[i][0] == xy_pos[i+1][0]):
				x_end = xy_pos[i+1][0]
				y_end = xy_pos[i+1][1]
				i += 1
		if(i < len(xy_pos)-1 and xy_pos[i][1] == xy_pos[i+1][1]):
			while(i < len(xy_pos)-1 and xy_pos[i][1] == xy_pos[i+1][1]):
				x_end = xy_pos[i+1][0]
				y_end = xy_pos[i+1][1]
				i += 1
		i += 1
		
		#draw line
		target_pose.position.x = start_x + (x_end - x_start) * scale_factor
		target_pose.position.y = start_y + (y_end - y_start) * scale_factor
		target_pose.position.z = pen_down
		print(target_pose.position)
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)

	#lift pen up after line is drawn
	target_pose.position.z = pen_up
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)


	'''
	xy_len = len(xy_pos)
	for i in range(xy_len):
		target_pose.position.x = start_x + xy_pos[i][0]*scale_factor
		target_pose.position.y = start_y + xy_pos[i][1]*scale_factor
		target_pose.position.z = pen_down
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
			return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		time.sleep(stall_time)
	# target_pose.position.x = start_x + xy_pos[xy_len-1][0]*scale_factor
	# target_pose.position.y = start_y + xy_pos[xy_len-1][1]*scale_factor
	target_pose.position.z = pen_up
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	time.sleep(stall_time)



def main():
	root = Tk()
	root.title('Maze Solving Control Panel')
	frame = Frame(root)
	frame.pack() 
	botframe = Frame(root) 
	botframe.pack(side = BOTTOM) 

	redb = Button(frame, text = 'Solve Maze', fg ='red') 
	redb.pack(side = LEFT) 
	redb.config(command = rec_mz)

	greenb = Button(frame, text = 'Generate Maze', fg='brown') 
	greenb.pack(side = LEFT) 
	greenb.config(command = gen_mz)

	blueb = Button(frame, text ='Calibrate', fg ='blue') 
	blueb.pack(side = LEFT) 
	blueb.config(command = cal)

	blackb_open = Button(botframe, text ='Open Gripper', fg ='black') 
	blackb_open.pack(side = BOTTOM) 
	blackb_open.config(command = ogripper)

	blackb_close = Button(botframe, text ='Close Gripper', fg ='black') 
	blackb_close.pack(side = BOTTOM) 
	blackb_close.config(command = cgripper)

	root.mainloop() 


if __name__ == "__main__":
	init()
	main()
