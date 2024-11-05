#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND
from config import LEFT_HOOK, RIGHT_HOOK
import time

from tools import collision, setcubeplacement, getcubeplacement, setupwithmeshcat, jointlimitsviolated
import math
from collections import deque
from pinocchio.utils import rotate

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    from inverse_geometry import computeqgrasppose
    robot, cube, viz = setupwithmeshcat()
    def edge_valid(se3_0, se3_1, steps=100):
       # Initialize list of configurations
       q_list = []
       q_new, found_grasp = computeqgrasppose(robot, robot.q0, cube, se3_0)
       q_list.append(q_new)
       step_size = (se3_1.translation - se3_0.translation) * (1 / steps)

       # Linear interpolation checking for grasp pose
       for i in range(1, steps):
           se3_translation = se3_0.translation + i * step_size
           se3_new = pin.SE3(rotate('z', 0), se3_translation)
           q_new, found_grasp = computeqgrasppose(robot, q_list[i-1], cube, se3_new)
        #    print("Interpolation step", i)
        #    print(se3_new.translation)
           if collision(robot, q_new):
               # print("------------------collision-----------------")
               return [], False
           if not found_grasp:
               return q_list, False
           q_list.append(q_new)

       return q_list, True
    
    def generateQRand(vertex, magnitude=0.1):
        valid_direction = False

        while not valid_direction:
            q, cube_placement = vertex
            random_direction = np.random.rand(3)
            random_direction /= np.linalg.norm(random_direction)
            perturbation = random_direction * magnitude
            random_direction = cube_placement.translation + perturbation

            #print("Random Direction", random_direction)
            valid_direction = random_direction[0] > 0 and random_direction[0] < 0.5 and random_direction[1] > -1 and random_direction[1] < 1 and random_direction[2] > 0.9 and random_direction[2] < 5.9
            #print(valid_direction)

        se3_new = pin.SE3(rotate('z', 0), random_direction)
        q_new, found_grasp = computeqgrasppose(robot, q, cube, se3_new)

        # x = np.random.rand() / 2
        # y = np.random.rand() - 0.5
        # z = np.random.rand() + 0.9
        # random_direction = np.array([x, y, z])
        # se3_new = pin.SE3(rotate('z', 0), random_direction)
        # q_new, found_grasp = computeqgrasppose(robot, q, cube, se3_new)

        return (q_new, se3_new), found_grasp 
    
    k = 100000
    vertices = [(qinit, cubeplacementq0)]
    edges = []
    path_edges = []

    for i in range(k):
        if i % 10 == 0:
            print("Iteration number:")
            print(i)
            print("Vertices:")
            print(len(vertices))
        qnear = vertices[np.random.randint(len(vertices))]
        qrand, found_grasp = generateQRand(qnear)
        while not found_grasp or collision(robot, qrand[0]):
            qrand, found_grasp = generateQRand(vertices[np.random.randint(len(vertices))])

        # viz.display(qrand[0])
        # time.sleep(0.3)
        
        qlist, e_v = edge_valid(qnear[1], qrand[1])
        if e_v:
            vertices.append(qrand)
            edges.append((qnear, qrand, qlist))
            qlist, e_v = edge_valid(qrand[1], cubeplacementqgoal)
            
            # print(found_grasp)
            # goal_edge_valid(qrand[0], qgoal)
            if e_v:
                print("Connected")
                edges.append((qrand, (qgoal, cubeplacementqgoal), qlist))
                break
        if i == k - 1:
            print("Failed to reach goal")
            return []
    
    print("Start Traceback")
    # Trace back from goal to start to find path edges
    current = (qgoal, cubeplacementqgoal)
    n = 0
    print(current[0])
    print(qinit)
    while list(current[0]) != list(qinit):
        print(current[0])
        print(qinit)
        n += 1
        if n % 20 == 0:
            print("Traceback iteration", n)
        for edge in edges:
            if list(edge[1][0]) == list(current[0]):
                print(n, "Edge found")
                path_edges.insert(0, edge[2])  # Prepend the qlist for each edge in the path
                current = edge[0]
                break
        else:
            print("Path not found")
            return []

    # Flatten the list of qlists along the path
    path = [q for qlist in path_edges for q in qlist]
    print("Path:", len(path))
    path.append(qgoal)
    print(path)
    return path


#     m = 1

#     # Define two sets of edges and vertices for two trees
#     edges_start, edges_goal = [], []
#     vertices_start, vertices_goal = [qinit], [qgoal]
#     k = 1000000

#     # Dictionaries to track connections for path reconstruction
#     adjacency_list_start = {tuple(qinit): []}
#     adjacency_list_goal = {tuple(qgoal): []}

#     def random_perturbation(point, magnitude=1): # TODO: add grasp position check
#         # # Generate a random unit vector in 15 dimensions and scale it by magnitude
#         # random_direction = np.random.randn(15)
#         # random_direction /= np.linalg.norm(random_direction)  # Normalize to unit length
#         # perturbation = random_direction * magnitude
#         # return point + perturbation

#         # generate random cube position (possibly change this to do a perturbation rather than completely random)
#         # inverse geometry to find q
#         # collision check and check edge; if fails, generate new point
#         # add to graph
#         return

#     def extend_tree(tree_vertices, tree_edges, adjacency_list, qtarget, magnitude=1):
#         # Choose a random existing vertex in the tree
#         qnear = tree_vertices[np.random.randint(len(tree_vertices))]

#         # Generate a new point by adding a random vector of magnitude m to qnear
#         qrand = random_perturbation(qnear, magnitude)
        
#         # Ensure the point is within valid bounds
#         qrand = np.clip(qrand, -np.pi, np.pi)

#         # Check if the edge from qnear to qrand is valid
#         if not collision(robot, qrand) & check_grasp_position(qrand, cubeplacementq0):
#             if edge_valid(qrand, qnear):
#                 tree_vertices.append(qrand)
#                 tree_edges.append((qnear, qrand))
#                 adjacency_list.setdefault(tuple(qnear), []).append(tuple(qrand))
#                 adjacency_list.setdefault(tuple(qrand), []).append(tuple(qnear))
                
# #                 # Check if we ca
# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
    
#     robot, cube, viz = setupwithmeshcat()
    
    
#     q = robot.q0.copy()
#     q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
#     qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
#     #print(check_grasp_position(q0, cube))
    
#     if not(successinit and successend):
#         print ("error: invalid initial or end configuration")
    
#     path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
#     displaypath(robot,path,dt=0.01,viz=viz) #you ll probably want to lower dt
    
#         # if i % 1000 == 0:
#         #     print(i)
#         # Grow the start tree toward a configuration modified by random vector
#         qconnect_start, success = extend_tree(vertices_start, edges_start, adjacency_list_start, qgoal, m)
#         if len(vertices_start) > start_tree_size:
#             start_tree_size = len(vertices_start)
#             print(i, "start tree addition, new size:", start_tree_size)
#         if success:
#             # Trees are connected, path can be constructed
#             vertices_start.append(qgoal)
#             edges_start.append((qconnect_start, qgoal))
#             adjacency_list_start.setdefault(tuple(qconnect_start), []).append(tuple(qgoal))
#             adjacency_list_start.setdefault(tuple(qgoal), []).append(tuple(qconnect_start))
#             break

#         # Grow the goal tree toward a configuration modified by random vector
#         qconnect_goal, success = extend_tree(vertices_goal, edges_goal, adjacency_list_goal, qconnect_start, m)
#         if len(vertices_goal) > goal_tree_size:
#             goal_tree_size = len(vertices_goal)
#             print(i, "goal tree addition, new size:", goal_tree_size)
#         if success:
#             # Trees are connected, path can be constructed
#             vertices_goal.append(qconnect_start)
#             edges_goal.append((qconnect_goal, qconnect_start))
#             adjacency_list_goal.setdefault(tuple(qconnect_goal), []).append(tuple(qconnect_start))
#             adjacency_list_goal.setdefault(tuple(qconnect_start), []).append(tuple(qconnect_goal))
#             break

#     # Path reconstruction from adjacency lists
#     path = []

#     def backtrack_path(adjacency_list, start, goal):
#         from collections import deque
#         queue = deque([[tuple(start)]])
#         visited = set([tuple(start)])
        
#         while queue:
#             current_path = queue.popleft()
#             current_node = current_path[-1]
#             if current_node == tuple(goal):
#                 return [list(config) for config in current_path]  # Convert to list format
            
#             for neighbor in adjacency_list.get(current_node, []):
#                 if neighbor not in visited:
#                     visited.add(neighbor)
#                     queue.append(current_path + [neighbor])
#         return []

#     # Build the path by combining the paths from both trees
#     path_start = backtrack_path(adjacency_list_start, qinit, qconnect_start)
#     path_goal = backtrack_path(adjacency_list_goal, qgoal, qconnect_start)
#     path = path_start + path_goal[::-1]  # Reverse goal path and concatenate

#     print("path length:")
#     print(len(path))
    
#     full_path = []
#     print("Distances:")
#     for i in range(1, len(path)):
#         start = np.array(path[i - 1])
#         end = np.array(path[i])
#         distance = np.linalg.norm(end - start)
#         if distance > 0.001:
#             print(distance)
#             q_step = (np.array(end) - start) * 0.001
#             for j in range(0, 1000):
#                 full_path.append(np.array(start) + j * q_step)

#     # Append the final point to ensure full_path ends exactly at path[-1]
#     full_path.append(np.array(path[-1]))

#     print("full path computed")
#     print("full path length:", len(full_path))
#     return full_path

# # Check that for a given robot configuration and cube placement, whether the cube is grasped by the robot
# def check_grasp_position(q, cube_placement):
#     pin.framesForwardKinematics(robot.model,robot.data,q)
#     pin.computeJointJacobians(robot.model,robot.data,q)

#     #Left Side
#     frameidL = robot.model.getFrameId(LEFT_HAND)
#     oMframeL = robot.data.oMf[frameidL]

#     oMcubeL = getcubeplacement(cube, LEFT_HOOK)

#     effL_XYZQUAT = pin.SE3ToXYZQUAT(oMframeL)
#     cubeL_XYZQUAT = pin.SE3ToXYZQUAT(oMcubeL)

#     #Right Side
#     frameidR = robot.model.getFrameId(RIGHT_HAND)
#     oMframeR = robot.data.oMf[frameidR]

#     oMcubeR = getcubeplacement(cube, RIGHT_HOOK)

#     effR_XYZQUAT = pin.SE3ToXYZQUAT(oMframeR)
#     cubeR_XYZQUAT = pin.SE3ToXYZQUAT(oMcubeR)

#     # print("Grasp position difference:")
#     # print(np.linalg.norm(effL_XYZQUAT - effR_XYZQUAT) - np.linalg.norm(cubeL_XYZQUAT - cubeR_XYZQUAT))
#     return ((np.linalg.norm(effL_XYZQUAT - effR_XYZQUAT) - np.linalg.norm(cubeL_XYZQUAT - cubeR_XYZQUAT)) < 0.00000000000001)

# def goal_edge_valid(q0, q1):
#     q_step = (q1 - q0) * 0.001
#     res = True
#     es = ""
#     print("Goal edge valid:")
#     for i in range(0, 1001):
#         if collision(robot, q0 + i * q_step):
#             es += "1"
#             res = False
#         else:
#             es += "0"
#     print(res)
#     print(es)
#     return res

def save_arrays_to_python_file(file_path, array_list):
    """
    Saves a list of numpy arrays to a Python file, making it importable.

    Parameters:
    file_path (str): Path where the .py file will be saved.
    array_list (list): List of numpy arrays to be saved.
    """
    with open(file_path, 'w') as file:
        # Write imports to file
        file.write("import numpy as np\n\n")
        
        # Write arrays to file
        for i, arr in enumerate(array_list):
            file.write(f"array_{i} = np.array({arr.tolist()})\n")
        
        # Optionally, save the entire list as a collection
        file.write("\narrays = [")
        for i in range(len(array_list)):
            file.write(f"array_{i}")
            if i < len(array_list) - 1:
                file.write(", ")
        file.write("]\n")

        file.write("def get_path():\n\treturn arrays")


def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose

    from path_example import get_path
    
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()

    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    #print(check_grasp_position(q0, cube))
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    # path = get_path()

    save_arrays_to_python_file('path_example.py', path)
    
    displaypath(robot,path,dt=0.01,viz=viz) #you ll probably want to lower dt
    