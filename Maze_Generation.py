# Maze Generation algorithm

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado


import numpy as np
import matplotlib.pyplot as plt


#only generates ODD mazes (i.e. 25x25 maze)
# takes width (n) and height (m)
def generate_maze(n, m):
    #if width or height are even, make them odd
    n1 = n
    m1 = m
    if(n%2 == 0):
        n1 = n+1
    if(m%2 == 0):
        m1 = m + 1
    
    
    innerMaze1 = (n1-2) // 2 + 1
    innerMaze2 = (m1-2) // 2 + 1
    #generate maze foundation
    maze = np.tile([[1, 2], [2, 0]], (innerMaze1, innerMaze2))

    #get list of walls coords and cell coords
    maze = maze[:-1, :-1]
    cells = {(i, j): (i, j) for i, j in np.argwhere(maze == 1)}
    walls = np.argwhere(maze == 2)

    def find(p, q):
        if p != cells[p] or q != cells[q]:
            cells[p], cells[q] = find(cells[p], cells[q])
        return cells[p], cells[q]

    #generate random maze
    np.random.shuffle(walls)
    for wi, wj in walls:
        if wi % 2:
            p, q = find((wi - 1, wj), (wi + 1, wj))
        else:
            p, q = find((wi, wj - 1), (wi, wj + 1))
        maze[wi, wj] = p != q

        if p != q:
            cells[p] = q

    #add outer walls to maze
    maze = maze.tolist()
    for i in range(len(maze)):
        maze[i].insert(0,0)
        maze[i].append(0)
    temp = []
    for i in range(m1):
        temp.append(0)
    maze.insert(0, temp)
    maze.append(temp)
    maze = np.array(maze)
    
    #reverse walls to be 1s and cells to be 0s
    for i in range(len(maze)):
        for j in range(len(maze[i])):

            if(maze[i][j] == 1):
                maze[i][j] = 0
            else:
                maze[i][j] = 1
    #add entrance and exit points to maze
    maze[0][1] = 0
    maze[(len(maze)-1)][(len(maze[0])-2)] = 0
    maze = maze.tolist()

    #show image of maze
    '''
    plt.imshow(maze, cmap=plt.cm.binary, interpolation='nearest')
    plt.xticks([]), plt.yticks([])
    plt.show()
    '''
    
    return maze
