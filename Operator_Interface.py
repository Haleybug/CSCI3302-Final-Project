# Operator Interface

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado


### sudo apt-get install python-tk


#import Camera_Interface
#import Maze_Generation
#import Maze_Solver
#import Image_Processor
#import Coordinate_Converter

import copy
from Tkinter import *
import rospy
import intera_interface
from sawyer_pykdl import sawyer_kinematics


def rec_mz():
	print("Taking picture of maze.\n")
	#maze = Camera_Interface()
	#xy_list = Maze_Solver(maze)
	xy_list = [[0,0],[0,1],[0,2],[1,2],[2,2]] # test path
	move_arm_sol_mz(xy_list)
	

def gen_mz():
	print("Generating a maze.\n")
	#Maze_Generation()
	#move_arm_gen_mz()

def cal():
	print("Calibrating corners of maze.\n")

def bbbbbbbb():
	print("Doing nothing.\n")





def move_arm_gen_mz():
	rospy.init_node("gen_mz")
	limb = intera_interface.Limb('right')
	kinematics = sawyer_kinematics('right')
	xyz_pos = [0.58, -0.48, 0.216]
	x = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos)))
	limb.move_to_joint_positions(x)


#def move_arm_sol_mz(xyz_pos): # xyz_pos = [0.58, -0.48, 0.216]
#	rospy.init_node("sol_mz")
#	limb = intera_interface.Limb('right')
#	kinematics = sawyer_kinematics('right')
#	dest_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos)))
#	limb.move_to_joint_positions(dest_pos)

def move_arm_sol_mz(xy_pos): # xy_pos = list of x,y coordinates following maze node order
	rospy.init_node("sol_mz")
	limb = intera_interface.Limb('right')
	kinematics = sawyer_kinematics('right')
	xyz_pos = copy.copy(xy_pos)
	xyz_pos = [xy_pos + [-0.02] for xy_pos in xyz_pos]
	xyz_pos[0][2] = 1
	xyz_len = len(xyz_pos)
	for i in range(0, xyz_len):
		next_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(xyz_pos[i])))
		limb.move_to_joint_positions(next_pos)
		if i == 0:
			pen_down = [xyz_pos[0][0], xyz_pos[0][1], xyz_pos[0][2]-1.02]
			next_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(pen_down)))
			limb.move_to_joint_positions(next_pos)
	pen_up = [xyz_pos[xyz_len][0], xyz_pos[xyz_len][1], xyz_pos[xyz_len][2]+1.02]
	end_pos = dict(zip(limb._joint_names, kinematics.inverse_kinematics(pen_up)))
	limb.move_to_joint_positions(end_pos)















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

blackb = Button(botframe, text ='Black', fg ='black') 
blackb.pack(side = BOTTOM) 
blackb.config(command = bbbbbbbb)

root.mainloop() 

