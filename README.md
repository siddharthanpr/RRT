# RRT
RRT - plugin for unidirectional and multi-directional RRT usin kd-trees with weighted euclidean.

run RRT.py to see unidirectional RRT planning from initial to goal configuration. 

the implementation uses a kd tree which does nearest neighbor search using weighted euclidean metric by seeing if a splittind axis 
is in an ellipsoid (instead of sphere)
