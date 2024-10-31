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
    # Performs linear interpolation while ensuring valid configurations with grasp poses
    def edge_valid(se3_0, se3_1, steps=10):
        # Initialise list of configurations
        q_list = []
        q_new, found_grasp = computeqgrasppose(robot, robot.q0, cube, se3_0) # Will succeed, not optimal as this has already been calculated
        #print(se3_0)
        q_list.append(q_new)
        step_size = (se3_1.translation - se3_0.translation) * (1 / steps)

        # Linear interpolation checking for grasp pose
        for i in range(1, steps):
            se3_translation = se3_0.translation + step_size
            se3_new = pin.SE3(rotate('z', 0), se3_translation)
            #se3_new.rotation = (1 - (i / steps)) * se3_0.rotation + (i / steps) * se3_1.rotation
            #se3_new.translation = se3_0.translation + (i / steps) * se3_1.translation
            #print(se3_new)
            q_new, found_grasp = computeqgrasppose(robot, q_list[i-1], cube, se3_new)
            if not found_grasp:
                return q_list, False
            q_list.append(q_new)

        return q_list, True
    
    def generateQRand(cube_placement, magnitude=0.1):
        # Generate a random unit vector in 15 dimensions and scale it by magnitude
        random_direction = np.random.rand(3)

        # Set Translation
        random_direction /= np.linalg.norm(random_direction)  # Normalize to unit length
        perturbation = random_direction * magnitude
        random_direction = cube_placement.translation + perturbation

        se3_new = pin.SE3(rotate('z', 0), random_direction)

        return se3_new

    k = 100     # Increase k for higher chance of find path if it exists; Will increase run time
    se3vertices = [cubeplacementq0]
    se3edges = []
    qvertices = [qinit]
    qedges = []
    # init_vertices = [cubeplacementq0]
    # goal_vertices = [cubeplacementqgoal]

    # init_edges = []
    # goal_edges = []


    # Build RRT; if connection is possible, it will be generated with large enough k
    for i in range(k):
        rand_choice = np.random.randint(len(se3vertices))
        se3_rand = generateQRand(se3vertices[rand_choice])
        q_rand, b = computeqgrasppose(robot, qvertices[rand_choice], cube, se3_rand)
        if not collision(robot, q_rand):
            if not jointlimitsviolated(robot, q_rand):
                # Nearest Neighbour
                min_dist = np.linalg.norm(qvertices[0] - q_rand)
                nearest_neighbour = 0
                for j in range(0, len(qvertices)):
                    dist = np.linalg.norm(qvertices[j] - q_rand)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_neighbour = j
                
                q_list, valid = edge_valid(se3vertices[nearest_neighbour], se3_rand)
                if valid:
                    qvertices.append(q_rand)
                    qedges.append([qvertices[nearest_neighbour], q_list, q_rand])
                    se3vertices.append(se3_rand)
                    se3edges.append([se3vertices[nearest_neighbour], se3_rand])
                    
                    # Check if goal edge is possible
                    q_list, valid = edge_valid(se3_rand, cubeplacementqgoal)
                    if valid:
                        print("Successfully found goal!!!")
                        qedges.append([q_rand, q_list, qgoal])
                        se3edges.append([se3_rand, cubeplacementqgoal])

    # Path find through the graph; BFS
    print(len(qvertices))
    queue = [0]
    print(queue)
    explored = []
    goal = qgoal
    parent_list = [None] * len(qvertices)   # Stores parent's index position for the current index position in vertices
    
    while queue:
        print("queued")
        v = queue.pop(0)
        if (qvertices[v] == goal).all():
            break
        for qedge in qedges:
            if (qvertices[v] == qedge[0]).all() | (qvertices[v] == qedge[2]).all():
                # Determine the adjacent vertex
                adjacent = qedge[2] if (qvertices[v] == qedge[0]).all() else qedge[0]
                
                in_explored = False
                for explored_qvertice in explored:
                    if (adjacent == explored_qvertice).all():
                        in_explored = True
                
                if not in_explored:
                    # Mark the adjacent vertex as explored
                    explored.append(adjacent)
                    # Add the adjacent vertex to the queue for further exploration
                    # Track the parent of the adjacent vertex
                    for qvertice_index in range(0, len(qvertices)):
                        if (qvertices[qvertice_index] == adjacent).all():
                            parent_list[qvertice_index] = v
                            queue.append(qvertice_index)

    # Reverse the BFS from goal
    print(parent_list)
    inverted_path = [qgoal]
    vertice = parent_list[1]
    inverted_path.append(parent_list[1])
    
    while vertice != goal:
        next_vertice = parent_list[parent_list.index(vertice)]
        inverted_path.append(parent_list[parent_list.index(vertice)])
        edge = next_vertice

    # Invert the path
    path = []*len(inverted_path)

    for i in range(len(inverted_path), 0):
        path[len(inverted_path) - i] = inverted_path[i]

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
                
#                 # Check if we can connect to qtarget (other tree)
#                 if edge_valid(qrand, qtarget):
#                     print("Connected!")
#                     return qrand, True  # Return the connection point
#         return qrand, False

#     start_tree_size = 1
#     goal_tree_size = 1
#     for i in range(k):
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


def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    #print(check_grasp_position(q0, cube))
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.001,viz=viz) #you ll probably want to lower dt
    