# Maze Solver algorithm

# Michael Gini, Justin Davis, Scott Weygandt, Haley Drexel, Oscar Delgado

import networkx as nx

# G is a Unweighted, Bidirectional NetworkX graph of the maze
# start is the beginning node (maze start)
# end is the end node (maze exit)

def solveMaze(G, start, end):
  
  path = nx.bidirectional_shortest_path(G, start, end)
  
  return path
