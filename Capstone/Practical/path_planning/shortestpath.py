#!/usr/bin/python

import numpy as np
import yaml
import math
from operator import itemgetter
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
    print "occ_map: "
    pprint.pprint(occ_map)

    # Converge start and goal positions to map indices.
    x = int(math.ceil((start.item(0) / x_spacing) - 0.5))  # startingx
    y = int(math.ceil((start.item(1) / y_spacing) - 0.5))  # startingy
    goalX = int(math.ceil((goal.item(0) / x_spacing) - 0.5))
    goalY = int(math.ceil((goal.item(1) / y_spacing) - 0.5))
    print "Start Pose: ", x, y
    print "Goal Pose: ", goalX, goalY

    # Make a map to keep track of all the nodes and their cost distance values.
    possible_nodes = [[0 for row in range(len(occ_map[0]))] for col in range(len(occ_map))]
    row = y
    col = x

    # Show the starting node and goal node.
    # 5 looks similar to S and 6 looks similar to G.
    possible_nodes[row][col] = 5
    # possible_nodes[goalY][goalX] = 6

    print "Possible Nodes: "
    pprint.pprint(possible_nodes)

    # The g_value will count the number of steps each node is from the start.
    # Since we are at the start node, the total cost is 0.
    g_value = 0
    frontier_nodes = [(g_value, col, row)] # dist, x, y
    searched_nodes = []
    parent_node = {}  # Dictionary that Maps {child node : parent node}
    loopcount = 0

    while len(frontier_nodes) != 0:
        print "\n>>>>>>>>>>>>LOOP COUNT: ", loopcount, "\n"
        frontier_nodes.sort(reverse=True) #sort from shortest distance to farthest
        current_node = frontier_nodes.pop()
        print "current_node: ", current_node
        print "frontier nodes: ", searched_nodes
        if current_node[1] == goalX and current_node[2] == goalY:
            print " %%%%%%%%%%%%% Goal found!"
            print "NEAREST NODE: ", current_node
            print "searched_nodes: \n", searched_nodes
            print "\n"
            print sorted(searched_nodes, key = itemgetter(0))
            break
        g_value, col, row = current_node
        print "current g, col, row:", g_value, col, row

        # Check surrounding neighbors.
        for i in delta:
            possible_expansion_x = col + i[0]
            possible_expansion_y = row + i[1]
            print "map dims: ", len(occupancy_map[0]), len(occ_map)
            valid_expansion = 0 <= possible_expansion_y < len(occupancy_map[0]) and 0 <= possible_expansion_x < len(occ_map)
            print "Current expansion Node: ", possible_expansion_x, possible_expansion_y

            if valid_expansion:
                try:
                    unsearched_node = possible_nodes[possible_expansion_y][possible_expansion_x] == 0
                    open_node = occ_map[possible_expansion_y][possible_expansion_x] == 0
                    print "Check Open or Wall: ", occ_map[possible_expansion_y][possible_expansion_x]
                except:
                    unsearched_node = False
                    open_node = False
                if unsearched_node and open_node:
                    # Using  instead of 1 to make it easier to read This node has been searched.
                    # searched_row = possible_expansion_y
                    # searched_col = possible_expansion_x
                    possible_nodes[possible_expansion_y][possible_expansion_x] = 3
                    possible_node = (g_value + cost, possible_expansion_x, possible_expansion_y)
                    frontier_nodes.append(possible_node)
                    print "frontier_nodes:", frontier_nodes

                    # This now builds parent/child relationship
                    parent_node[possible_node] = current_node
                    print "Parent Node: \n", parent_node
                    print "While Possible Nodes: "
                    pprint.pprint(possible_nodes)
        loopcount = loopcount+1

    print "Generating path..."

    route = []
    child_node = current_node
    while parent_node.has_key(child_node):
        route.append(parent_node[child_node])
        child_node = parent_node[child_node]
        route.sort()

    #  route back to metric units:
    print "Route: ", route
    path = []
    position = [start.item(0), start.item(1)]  # Starting point passed in by function
    path.append(position)  # Add it to the list for the path

    for i in range(0, len(route)):
        position = [round((route[i][1]+0.5)*x_spacing, 3), round((route[i][2]+0.5)*y_spacing, 3)]
        path.append(position)

    # Add the goal state:

    position = [goal.item(0), goal.item(1)]
    path.append(position)

    print "Path: "
    pprint.pprint(path)

    # Convert to numpy array and return.
    path = np.array(path)
    return path


def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        # [ 0.3  ,  0.3  ],  # [2,1]
        # [ 0.325,  0.3  ],  # [2,1]
        # [ 0.325,  0.5  ],  # [2,2]
        # [ 0.325,  0.7  ],  # [2,3]
        # [ 0.455,  0.7  ],  # [3,3]
        # [ 0.455,  0.9  ],  # [3,4]
        # [ 0.585,  0.9  ],  # [4,4]
        # [ 0.600,  1.0  ]   # [5,5]

         [0.3, 0.3],
         [0.325, 0.3],
         [0.325, 0.5],
         [0.325, 0.7],
         [0.325, 0.9],
         [0.325, 1.1],
         [0.455, 1.1],
         [0.585, 1.1],
         [0.6, 1.0]

    ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")
    else:
        print "nope, chuck testa"

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],  # [2, 5]
                           [ 0.5,  1.1],  # [2, 5]
                           [ 0.5,  1.3],  # [2, 6]
                           [ 0.5,  1.5],  # [2, 7]
                           [ 0.7,  1.5],  # [3, 7]
                           [ 0.9,  1.5],  # [4, 7]
                           [ 1.1,  1.5],  # [5, 7]
                           [ 1.1,  1.3],  # [5, 6]
                           [ 1.1,  1.1],  # [5, 5]
                           [ 1.1,  0.9]   # [5, 4]
                           ])


    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    t = 0
    for i in range(len(path1)-1):
      t += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(t)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 1 length:")
    print(t)
    print("Path 2 length:")
    print(s)



def main():
    test_for_grader()
    # # Load parameters from yaml
    # param_path = 'params.yaml' # rospy.get_param("~param_path")
    # f = open('params.yaml','r')
    # params_raw = f.read()
    # f.close()
    # params = yaml.load(params_raw)
    # # Get params we need
    # occupancy_map = np.array(params['occupancy_map'])
    # pos_init = np.array(params['pos_init'])
    # pos_goal = np.array(params['pos_goal'])
    # x_spacing = params['x_spacing']
    # y_spacing = params['y_spacing']
    # path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    # print(path)

if __name__ == '__main__':
    main()

