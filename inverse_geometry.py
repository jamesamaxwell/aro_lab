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
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from tools import setcubeplacement

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    #TODO implement
    #print ("TODO: implement me")
    #Plan: Use BFGS to find solution

    def callback(q):
        time.sleep(.5)

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

        if collision(robot, q):
            return 10000 + norm(effL_XYZQUAT - cubeL_XYZQUAT) ** 2 + norm(effR_XYZQUAT - cubeR_XYZQUAT) ** 2

        return norm(effL_XYZQUAT - cubeL_XYZQUAT) ** 2 + norm(effR_XYZQUAT - cubeR_XYZQUAT) ** 2

    print(cost(qcurrent))
    qopt_bfgs = fmin_bfgs(cost, qcurrent, callback=callback)
    robot.q0 = qopt_bfgs
    return robot.q0, True
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, qe)
    
    
    
