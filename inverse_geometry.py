#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np

from numpy.linalg import pinv,inv,norm,svd,eig
from scipy.optimize import fmin_bfgs
from tools import collision, getcubeplacement, setcubeplacement, jointlimitscost
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from config import EPSILON

from tools import setcubeplacement

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
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

        constraints = np.array([jointcost])
        return constraints


    def penalty(q):
        return cost(q) + 10 * sum(np.square(constraint(q)))

    # Use fmin_bfgs to find solution
    qopt_bfgs = fmin_bfgs(penalty, qcurrent, callback=callback, disp=0)
    
    return qopt_bfgs, (cost(qopt_bfgs) < EPSILON) & (not collision(robot, qopt_bfgs))
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q)
    