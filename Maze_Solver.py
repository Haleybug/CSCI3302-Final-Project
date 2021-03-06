# Maze Solver algorithm
# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado

from Maze_Generation import *
import numpy as np
import copy
        
#check if it the coordinate is not a wall
def isSafe( maze, x, y, n): 
      
    if x >= 0 and x < n and y >= 0 and y < n and maze[x][y] == 0: 
        return True
      
    return False

      
# maze solving helper
def solveMazeUtil(maze, x, y, x_end, y_end, sol, n): 
      
    #has goal been reached
    if x == x_end and y == y_end: 
        sol[x][y] = 0
        return True
          
   
    if isSafe(maze, x, y, n) == True:
        # print((x, y))
        sol[x][y] = 0
        #mark node has having already been visited
        maze[x][y] = 2
          
        #check if surroundings have a viable path
        if solveMazeUtil(maze, x + 1, y, x_end, y_end, sol, n) == True:
            return True
              
        if solveMazeUtil(maze, x, y + 1, x_end, y_end,sol, n) == True: 
            return True
        if solveMazeUtil(maze, x-1, y, x_end, y_end,sol, n) == True: 
            return True
        if solveMazeUtil(maze, x, y-1, x_end, y_end,sol, n) == True: 
            return True
          
        # If none of the above movements work then  
        # BACKTRACK: unmark x, y as part of solution path 
        sol[x][y] = 1
        
        return False

# call this function to solve a maze and get the coordinates for path
# takes in SQUARE maze
# takes in x,y coordinates of the start and the x,y coordinates of a valid ending position
# takes in width/height of square (n)
# 1's are walls, 0's are path
def solveMaze( maze, x_start, y_start, x_end, y_end, n): 

    #generate solution with all walls
    sol = [ [ 1 for j in range(len(maze)) ] for i in range(len(maze)) ]
      
    if solveMazeUtil(maze, x_start, y_start, x_end, y_end, sol, n ) == False: 
        print("Solution doesn't exist"); 
        return []
      
    #print(np.array(sol))
    coords = []
    #create list of coordinates for path
    for i in range(len(sol)):
        for j in range(len(sol[0])):
            if(sol[i][j] == 0):
                coords.append((i, j))

    #return list of coordinates
    return coords

def isValid(maze,x,y,h,w):
    if x < 0:
        return False
    if y < 0:
        return False
    if x >= w:
        return False
    if y >= h:
        return False
    return (maze[y][x] == 0)

#Since all transitions have a cost of 1, BFS and djikstra are equivalent
#coordinates are (x,y)
def solveMaze2(maze, start, end, h, w):
    explored = copy.copy(maze)

    fringe = []
    fringe.append((start, []))

    while len(fringe) > 0:
        node, path = fringe.pop()
        if maze[node[1]][node[0]] == 1:
            continue
        maze[node[1]][node[0]] = 1

        path.append(node)

        if node == end:
            sol = [(b,a) for a,b in path]
            return sol

        if isValid(maze, node[0], node[1]-1, h, w):
            fringe.append(((node[0], node[1]-1), copy.copy(path)))
        if isValid(maze, node[0], node[1]+1, h, w):
            fringe.append(((node[0], node[1]+1), copy.copy(path)))
        if isValid(maze, node[0]-1, node[1], h, w):
            fringe.append(((node[0]-1, node[1]), copy.copy(path)))
        if isValid(maze, node[0]+1, node[1], h, w):
            fringe.append(((node[0]+1, node[1]), copy.copy(path)))




# maze = [[1,1,1,1,1,1,1,1,1],
#         [1,0,1,0,0,0,1,0,1],
#         [1,0,1,0,1,0,1,0,1],
#         [1,0,0,0,1,0,0,0,1],
#         [1,0,1,1,1,0,1,1,1],
#         [1,0,0,0,1,0,0,0,1],
#         [1,1,1,1,1,1,1,1,1]]
# start = (1,1)
# end = (7,5)

# print(maze)

# path = solveMaze2(maze, start, end, 7, 9)

# print path

'''
if __name__ == "__main__":
    maze = generate_maze(24, 24)
    x_end = 0
    y_end = 0
    
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if(maze[i][j]== 0):
                x_end = i 
                y_end = j
    
    print(solveMaze(maze, 0, 1, x_end, y_end, len(maze)))
'''
