#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
import time

from tools import collision, setupwithmeshcat, distanceToObstacle
from pinocchio.utils import rotate

# Returns a collision free path from qinit to qgoal under grasping constraints
# The path is expressed as a list of configurations
def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    from inverse_geometry import computeqgrasppose
    robot, cube, viz = setupwithmeshcat()

    # Function to check edge is valid when interpolated
    def edge_valid(se3_0, se3_1, steps=10):
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

           if collision(robot, q_new):
               return q_list, False
           if not found_grasp:
               return q_list, False
           if (distanceToObstacle(robot, q_new) < min_dist_to_obstacle):
               return q_list, False
           
           q_list.append(q_new)

       return q_list, True
    
    # Generate a new SE3 point a magnitude away from vertex
    def generateQRand(vertex, magnitude=0.1):
        valid_direction = False

        while not valid_direction:
            q, cube_placement = vertex
            random_direction = np.random.rand(3)
            random_direction /= np.linalg.norm(random_direction)
            perturbation = random_direction * magnitude
            random_direction = cube_placement.translation + perturbation

            # Check direction was in bounding box
            valid_direction = (random_direction[0] > 0 and random_direction[0] < 0.5 and random_direction[1] > -1 and 
                                random_direction[1] < 1 and random_direction[2] > 0.9 and random_direction[2] < 5.9)

        se3_new = pin.SE3(rotate('z', 0), random_direction)
        q_new, found_grasp = computeqgrasppose(robot, q, cube, se3_new)

        return (q_new, se3_new), found_grasp 
    
    k = 100000
    vertices = [(qinit, cubeplacementq0)]
    edges = []
    path_edges = []

    # Minimum distance to obstacles, ensures certain distance away from obstacles
    min_dist_to_obstacle = 0.01

    # Modified RRT
    for i in range(k):
        if i % 10 == 0:
            print("Iteration number:")
            print(i)
            print("Valid Vertices:")
            print(len(vertices))

        qnear = vertices[np.random.randint(len(vertices))]
        qrand, found_grasp = generateQRand(qnear)

        while not found_grasp or collision(robot, qrand[0]) or (distanceToObstacle(robot, qrand[0]) < min_dist_to_obstacle):
            qrand, found_grasp = generateQRand(vertices[np.random.randint(len(vertices))])
        
        qlist, e_v = edge_valid(qnear[1], qrand[1])

        if e_v:
            vertices.append(qrand)
            edges.append((qnear, qrand, qlist))
            qlist, e_v = edge_valid(qrand[1], cubeplacementqgoal)
 
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

    while list(current[0]) != list(qinit):
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
    path.append(qgoal)
    print(path)
    return path

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
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,path,dt=0.1,viz=viz) #you ll probably want to lower dt
    
    
