# Operator Interface

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado


### sudo apt-get install python-tk
### pip install python-tk

#import Tkinter
from Tkinter import *
 


def rec_mz():
	print("Taking picture of maze.\n")
	

def gen_mz():
	print("Generating a maze.\n")

def cal():
	print("Calibrating corners of maze.\n")

def bbbbbbbb():
	print("Doing nothing.\n")


root = Tk()
root.title('Maze Solving Control Panel')
frame = Frame(root) 
frame.pack() 
botframe = Frame(root) 
botframe.pack(side = BOTTOM) 

redb = Button(frame, text = 'Record Maze', fg ='red') 
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

