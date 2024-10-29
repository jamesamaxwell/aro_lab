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
import time

from tools import collision, setupwithmeshcat
import math
from collections import deque

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    m = 1

    # Define two sets of edges and vertices for two trees
    edges_start, edges_goal = [], []
    vertices_start, vertices_goal = [qinit], [qgoal]
    k = 10000

    # Dictionaries to track connections for path reconstruction
    adjacency_list_start = {tuple(qinit): []}
    adjacency_list_goal = {tuple(qgoal): []}

    def random_perturbation(point, magnitude=1): # TODO: add grasp position check
        # Generate a random unit vector in 15 dimensions and scale it by magnitude
        random_direction = np.random.randn(15)
        random_direction /= np.linalg.norm(random_direction)  # Normalize to unit length
        perturbation = random_direction * magnitude
        return point + perturbation

    def extend_tree(tree_vertices, tree_edges, adjacency_list, qtarget, magnitude=1):
        # Choose a random existing vertex in the tree
        qnear = tree_vertices[np.random.randint(len(tree_vertices))]

        # Generate a new point by adding a random vector of magnitude m to qnear
        qrand = random_perturbation(qnear, magnitude)
        
        # Ensure the point is within valid bounds
        qrand = np.clip(qrand, -np.pi, np.pi)

        # Check if the edge from qnear to qrand is valid
        if not collision(robot, qrand):
            if edge_valid(qrand, qnear):
                tree_vertices.append(qrand)
                tree_edges.append((qnear, qrand))
                adjacency_list.setdefault(tuple(qnear), []).append(tuple(qrand))
                adjacency_list.setdefault(tuple(qrand), []).append(tuple(qnear))
                
                # Check if we can connect to qtarget (other tree)
                if edge_valid(qrand, qtarget):
                    print("Connected!")
                    return qrand, True  # Return the connection point
        return qrand, False

    start_tree_size = 1
    goal_tree_size = 1
    for i in range(k):
        # if i % 1000 == 0:
        #     print(i)
        # Grow the start tree toward a configuration modified by random vector
        qconnect_start, success = extend_tree(vertices_start, edges_start, adjacency_list_start, qgoal, m)
        if len(vertices_start) > start_tree_size:
            start_tree_size = len(vertices_start)
            print(i, "start tree addition, new size:", start_tree_size)
        if success:
            # Trees are connected, path can be constructed
            vertices_start.append(qgoal)
            edges_start.append((qconnect_start, qgoal))
            adjacency_list_start.setdefault(tuple(qconnect_start), []).append(tuple(qgoal))
            adjacency_list_start.setdefault(tuple(qgoal), []).append(tuple(qconnect_start))
            break

        # Grow the goal tree toward a configuration modified by random vector
        qconnect_goal, success = extend_tree(vertices_goal, edges_goal, adjacency_list_goal, qconnect_start, m)
        if len(vertices_goal) > goal_tree_size:
            goal_tree_size = len(vertices_goal)
            print(i, "goal tree addition, new size:", goal_tree_size)
        if success:
            # Trees are connected, path can be constructed
            vertices_goal.append(qconnect_start)
            edges_goal.append((qconnect_goal, qconnect_start))
            adjacency_list_goal.setdefault(tuple(qconnect_goal), []).append(tuple(qconnect_start))
            adjacency_list_goal.setdefault(tuple(qconnect_start), []).append(tuple(qconnect_goal))
            break

    # Path reconstruction from adjacency lists
    path = []

    def backtrack_path(adjacency_list, start, goal):
        from collections import deque
        queue = deque([[tuple(start)]])
        visited = set([tuple(start)])
        
        while queue:
            current_path = queue.popleft()
            current_node = current_path[-1]
            if current_node == tuple(goal):
                return [list(config) for config in current_path]  # Convert to list format
            
            for neighbor in adjacency_list.get(current_node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(current_path + [neighbor])
        return []

    # Build the path by combining the paths from both trees
    path_start = backtrack_path(adjacency_list_start, qinit, qconnect_start)
    path_goal = backtrack_path(adjacency_list_goal, qgoal, qconnect_start)
    path = path_start + path_goal[::-1]  # Reverse goal path and concatenate

    print("path length:")
    print(len(path))
    
    full_path = []
    print("Distances:")
    for i in range(1, len(path)):
        start = np.array(path[i - 1])
        end = np.array(path[i])
        distance = np.linalg.norm(end - start)
        if distance > 0.001:
            print(distance)
            q_step = (np.array(end) - start) * 0.001
            for j in range(0, 1000):
                full_path.append(np.array(start) + j * q_step)

    # Append the final point to ensure full_path ends exactly at path[-1]
    full_path.append(np.array(path[-1]))

    print("full path computed")
    print("full path length:", len(full_path))
    return full_path

def edge_valid(q0, q1):
    q_step = (q1 - q0) * 0.001
    for i in range(1, 1000):
        if collision(robot, q0 + i * q_step):
            return False
        
    return True

def check_grasp_position(q):
    
    return True

def goal_edge_valid(q0, q1):
    q_step = (q1 - q0) * 0.001
    res = True
    es = ""
    print("Goal edge valid:")
    for i in range(0, 1001):
        if collision(robot, q0 + i * q_step):
            es += "1"
            res = False
        else:
            es += "0"
    print(res)
    print(es)
    return res


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
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.001,viz=viz) #you ll probably want to lower dt
    
