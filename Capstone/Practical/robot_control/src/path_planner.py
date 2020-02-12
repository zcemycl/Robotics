#!/usr/bin/python

import numpy as np
import yaml
import math
from operator import itemgetter
import heapq
import pprint




def dijkstras(occupancy_map, x_spacing, y_spacing, start, goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # We will use this delta function to search surrounding nodes.
    delta = [[-1, 0],  # go up
             [0, -1],  # go left
             [1, 0],  # go down
             [0, 1]]  # go right

    # Each node on the map "costs" 1 step to reach.
    cost = 1
    # Convert numpy array of map to list of map, makes it easier to search.
    occ_map = occupancy_map.tolist()

    # Converge start and goal positions to map indices.
    x = int(math.ceil((start.item(0) / x_spacing) - 0.5))  # startingx
    y = int(math.ceil((start.item(1) / y_spacing) - 0.5))  # startingy
    goalX = int(math.ceil((goal.item(0) / x_spacing) - 0.5))
    goalY = int(math.ceil((goal.item(1) / y_spacing) - 0.5))
    # print "Start Pose: ", x, y
    # print "Goal Pose: ", goalX, goalY

    # Make a map to keep track of all the nodes and their cost distance values.
    possible_nodes = [[0 for row in range(len(occ_map[0]))] for col in range(len(occ_map[1]))]
    row = y
    col = x

    possible_nodes[row][col] = 1 #This means the starting node has been searched.
    # print "Possible Nodes: "
    # pprint.pprint(possible_nodes)

    # The g_value will count the number of steps each node is from the start.
    # Since we are at the start node, the total cost is 0.
    g_value = 0
    frontier_nodes = [(g_value, col, row)] # dist, x, y
    searched_nodes = []
    parent_node = {}  # Dictionary that Maps {child node : parent node}
    loopcount = 0

    while len(frontier_nodes) != 0:
        # print "\n>>>>>>>>>>>>LOOP COUNT: ", loopcount, "\n"
        frontier_nodes.sort(reverse=True) #sort from shortest distance to farthest
        current_node = frontier_nodes.pop()
        # print "current_node: ", current_node
        heapq.heappush(searched_nodes, current_node)
        # print "frontier nodes: ", searched_nodes
        if current_node[1] == goalX and current_node[2] == goalY:
            # print "Goal found!"
            # print "NEAREST NODE: ", current_node
            # print "searched_nodes: \n", searched_nodes
            # print "\n"
            # print sorted(searched_nodes, key = itemgetter(0))
            break
        g_value, col, row = current_node
        # print "current g, col, row:", g_value, col, row

        # Check surrounding neighbors.
        for i in delta:
            possible_expansion_x = col + i[0]
            possible_expansion_y = row + i[1]
            valid_expansion = 0 <= possible_expansion_x < len(occupancy_map[0]) and 0 <= possible_expansion_y < len(occ_map)
            # print "Current expansion Node: ", possible_expansion_x, possible_expansion_y

            if valid_expansion:
                try:
                    unsearched_node = possible_nodes[possible_expansion_x][possible_expansion_y] == 0
                    open_node = occ_map[possible_expansion_x][possible_expansion_y] == 0
                except:
                    unsearched_node = False
                if unsearched_node and open_node:
                    possible_nodes[possible_expansion_x][possible_expansion_y] = 1
                    possible_node = (g_value + cost, possible_expansion_x, possible_expansion_y)
                    frontier_nodes.append(possible_node)
                    # print "frontier_nodes:", frontier_nodes
                    # This now builds parent/child relationship
                    parent_node[possible_node] = current_node
                    # print "Parent Node: \n", parent_node
                    # print "While Possible Nodes: "
                    # pprint.pprint(possible_nodes)
        loopcount = loopcount+1

    # print "Generating path..."

    route = []
    child_node = current_node
    while parent_node.has_key(child_node):
        route.append(parent_node[child_node])
        child_node = parent_node[child_node]
        route.sort()

    # Convert route back to metric units:

    path = []
    position = [start.item(0), start.item(1)] #starting point passed in by function
    path.append(position) #add it to the list for the path

    for i in range(0, len(route)):
        position = [(route[i][1]+0.5)*x_spacing, (route[i][2]+0.5)*y_spacing ]
        path.append(position)

    # Add the goal state:

    position = [goal.item(0), goal.item(1)]
    path.append(position)

    # print "Pathh: "
    # pprint.pprint(path)
    path = np.array(path)
    return path

