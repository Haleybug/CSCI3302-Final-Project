# Operator Interface

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado


### sudo apt-get install python-tk


#import Camera_Interface
from Maze_Generation import *
#import Maze_Solver
#import Image_Processor
#import Coordinate_Converter

import copy
from Tkinter import *
import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics
from geometry_msgs.msg import Pose, Point, Quaternion

g_limb = None
g_orientation_hand_down = None
g_position_neutral = None
maze_dim_m = 31
maze_dim_n = 31


def rec_mz():
	print("Taking picture of maze.\n")
	#maze = Camera_Interface()
	#xy_list = Maze_Solver(maze)
	xy_list = [[0,0],[0,1],[0,2],[1,2],[2,2]] # test path
	move_arm_sol_mz(xy_list)
	

def gen_mz():
	print("Generating a maze.\n")
	generation_maze(maze_dim_m, maze_dim_n) # Takes in m,n for maze dimensions
	#move_arm_gen_mz()

def cal():
	global maze_dim_m, maze_dim_n
	print("Calibrating corners of maze.\n")
	maze_dim_m = 31
	maze_dim_n = 31

def ogripper():
	rospy.init_node('sawyer_ogripper')
	print("Openning gripper.\n")
	publisher_open = rospy.Publisher('/cairo/sawyer_gripper_open', Int16, queue_size=10)	
	publisher_open.publish()


def cgripper():
	rospy.init_node('sawyer_cgripper')
	print("Closing gripper.\n")
	publisher_close = rospy.Publisher('/cairo/sawyer_gripper_close', Int16, queue_size=10)
	publisher_close.publish()


def move_arm_gen_mz(xy_pos):
	global g_limb, g_position_neutral, g_orientation_hand_down
	init()
	
	scale_factor = 1
	grid_size = 1
	half_gird = grid_size/2

	g_limb.set_joint_position_speed(0.3)
	g_limb.move_to_neutral()
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
	target_pose.position.x = 0
	target_pose.position.y = 0 - half_grid
	target_pose.position.z = 1
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
	return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	target_pose.position.x = 0
	target_pose.position.y = 0 - half_grid
	target_pose.position.z = -0.02
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
	return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	
	xy_row_len = len(xy_pos)
	xy_col_len = len(xy_pos[0])
	for i in range(xy_row_len):
		for j in range(xy_col_len - 1):
			if xy_pos[i][j] == 1 and xy_pos[i][j+1] == 1: # Tuples???????????????
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor + half_grid
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 0 and xy_pos[i][j+1] == 0: 
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor + half_grid
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 0 and xy_pos[i][j+1] == 1:
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor - half_grid
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor - half_grid
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor + half_grid
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 1 and xy_pos[i][j+1] == 0:
				target_pose.position.x = i*scale_factor
				target_pose.position.y = j*scale_factor + half_grid
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = i*scale_factor
				target_pose.position.y = (j+1)*scale_factor + half_grid
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		target_pose.position.x = xy_row_len*scale_factor
		target_pose.position.y = (xy_col_len+1)*scale_factor + half_grid
		target_pose.position.z = 1
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	for j in range(xy_col_len):
		for i in range(xy_row_len - 1):
			if xy_pos[i][j] == 1 and xy_pos[i+1][j] == 1: # Tuples???????????????
				target_pose.position.x = (i+1)*scale_factor + half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 0 and xy_pos[i+1][j] == 0: 
				target_pose.position.x = (i+1)*scale_factor + half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 0 and xy_pos[i+1][j] == 1:
				target_pose.position.x = (i+1)*scale_factor - half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = (i+1)*scale_factor - half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = (i+1)*scale_factor + half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = -0.02
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
			if xy_pos[i][j] == 1 and xy_pos[i+1][j] == 0:
				target_pose.position.x = i*scale_factor + half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
				target_pose.position.x = (i+1)*scale_factor + half_grid
				target_pose.position.y = j*scale_factor
				target_pose.position.z = 1
				target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
				# The IK Service returns false if it can't find a joint configuration
				if target_joint_angles is False:
					rospy.logerr("Couldn't solve for position %s" % str(target_pose))
				return
				g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
		target_pose.position.x = (xy_row_len+1)*scale_factor + half_grid
		target_pose.position.y = xy_col_len*scale_factor
		target_pose.position.z = 1
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)


#def move_arm_gen_mz():
#	rospy.init_node("gen_mz")
#	limb = intera_interface.Limb('right')
#	kinematics = sawyer_kinematics('right')
#	xyz_pos = [0.58, -0.48, 0.216]
#	x = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos)))
#	limb.move_to_joint_positions(x)


#def move_arm_sol_mz(xyz_pos): # xyz_pos = [0.58, -0.48, 0.216]
#	rospy.init_node("sol_mz")
#	limb = intera_interface.Limb('right')
#	kinematics = sawyer_kinematics('right')
#	dest_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos)))
#	limb.move_to_joint_positions(dest_pos)

#def move_arm_sol_mz(xy_pos): # xy_pos = list of x,y coordinates following maze node order
#	rospy.init_node("sol_mz")
#	limb = intera_interface.Limb('right')
#	kinematics = sawyer_kinematics('right')
#	xyz_pos = copy.copy(xy_pos)
#	xyz_pos = [xy_pos + [-0.02] for xy_pos in xyz_pos]
#	xyz_pos[0][2] = 1
#	xyz_len = len(xyz_pos)
#	for i in range(0, xyz_len):
#		next_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos[i])))
#		limb.move_to_joint_positions(next_pos)
#		if i == 0:
#			pen_down = [xyz_pos[0][0], xyz_pos[0][1], xyz_pos[0][2]-1.02]
#			next_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(pen_down)))
#			limb.move_to_joint_positions(next_pos)
#	pen_up = [xyz_pos[xyz_len][0], xyz_pos[xyz_len][1], xyz_pos[xyz_len][2]+1.02]
#	end_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(pen_up)))
#	limb.move_to_joint_positions(end_pos)


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
	init()

	scale_factor = 1

	g_limb.set_joint_position_speed(0.3)
	g_limb.move_to_neutral()
	target_pose = Pose()
	target_pose.position = copy.deepcopy(g_position_neutral)
	target_pose.orientation = copy.deepcopy(g_orientation_hand_down)
	target_pose.position.x = xy_pos[0][0]*scale_factor
	target_pose.position.y = xy_pos[0][1]*scale_factor
	target_pose.position.z = 1
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
	return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)
	
	xy_len = len(xy_pos)
	for i in range(xy_len):
		target_pose.position.x = xy_pos[i][0]*scale_factor
		target_pose.position.y = xy_pos[i][1]*scale_factor
		target_pose.position.z = -0.02
		target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
		# The IK Service returns false if it can't find a joint configuration
		if target_joint_angles is False:
			rospy.logerr("Couldn't solve for position %s" % str(target_pose))
		return
		g_limb.move_to_joint_positions(target_joint_angles, timeout=2)

	target_pose.position.x = xy_pos[xy_len][0]*scale_factor
	target_pose.position.y = xy_pos[xy_len][1]*scale_factor
	target_pose.position.z = 1
	target_joint_angles = g_limb.ik_request(target_pose, "right_hand")
	# The IK Service returns false if it can't find a joint configuration
	if target_joint_angles is False:
		rospy.logerr("Couldn't solve for position %s" % str(target_pose))
	return
	g_limb.move_to_joint_positions(target_joint_angles, timeout=2)









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

	blackb = Button(botframe, text ='Open Gripper', fg ='black') 
	blackb.pack(side = BOTTOM) 
	blackb.config(command = ogripper)

	blackb = Button(botframe, text ='Close Gripper', fg ='black') 
	blackb.pack(side = BOTTOM) 
	blackb.config(command = cgripper)

	root.mainloop() 


if __name__ == "__main__":
	main()
