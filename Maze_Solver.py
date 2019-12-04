# Maze Solver algorithm

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado

import networkx as nx

def solveMaze(G, start, end):
  
  path = nx.bidirectional_shortest_path(G, start, end)
  
  return path
