#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
import time
from numpy.linalg import pinv,inv,norm,svd,eig
from scipy.optimize import fmin_bfgs
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, jointlimitscost, distanceToObstacle
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from tools import setcubeplacement

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    #Use BFGS to find solution

    # Basic callback function for BFGS optimiser
    def callback(q):
        # time.sleep(.5)
        pass

    # Cost function for BFGS, minimising the distance between the effectors and cube hooks
    def cost(q):
        pin.framesForwardKinematics(robot.model,robot.data,q)
        pin.computeJointJacobians(robot.model,robot.data,q)

        #Left Side
        frameidL = robot.model.getFrameId(LEFT_HAND)
        oMframeL = robot.data.oMf[frameidL]
    
        oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    
        effL_XYZQUAT = pin.SE3ToXYZQUAT(oMframeL)
        cubeL_XYZQUAT = pin.SE3ToXYZQUAT(oMcubeL)

        #Right Side
        frameidR = robot.model.getFrameId(RIGHT_HAND)
        oMframeR = robot.data.oMf[frameidR]
    
        oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    
        effR_XYZQUAT = pin.SE3ToXYZQUAT(oMframeR)
        cubeR_XYZQUAT = pin.SE3ToXYZQUAT(oMcubeR)

        return norm(effL_XYZQUAT - cubeL_XYZQUAT) ** 2 + norm(effR_XYZQUAT - cubeR_XYZQUAT) ** 2
    
    def constraint(q):
        # Constraint 1: Robot doesn't violate joint constraints
        jointcost = jointlimitscost(robot, q)   # Minimise joint cost

        # Ignore: Constraint 2 can be dealt with else where
        # Constraint 2: Robot isn't in collision with any objects
        #repulsive_component = collision_constraint(q)

        constraints = np.array([jointcost])
        return constraints
    
    def collision_constraint(q):
        collision_distance = distanceToObstacle(robot, q)   # p(x)
        max_influence_distance = 0.01 # p0(x)

        # Constraint Calculation
        n = 1
        if collision_distance <= max_influence_distance:
            Ur = 1/2 * n * np.square((1 / collision_distance) - (1 / max_influence_distance))
        else:
            Ur = 0

        return Ur


    def penalty(q):
        return cost(q) + 10 * sum(np.square(constraint(q)))

    qopt_bfgs = fmin_bfgs(penalty, qcurrent, callback=callback, disp=0)
    robot.q0 = qopt_bfgs

    print((cost(qopt_bfgs) < EPSILON), (not collision(robot, qopt_bfgs)))
    
    return robot.q0, (cost(qopt_bfgs) < EPSILON) & (not collision(robot, qopt_bfgs))
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()

    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)

    updatevisuals(viz, robot, cube, qtest)
    