# Maze Solver algorithm

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado

import networkx as nx

def SolveMaze(mazeGraph, start, end):
  
  path = nx.shortest_path(mazeGraph, start, end, weight='weight')
  
  return path
