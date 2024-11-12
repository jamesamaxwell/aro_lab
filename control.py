#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier
    
# These numbers work well for this particular solution.
Kp = 2500               # proportional gain (P of PD)
Kd = 1   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
 
    torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    
    # Given trajs (position, velocity and acceleration)
    # determine the required torque to move the robot
    q_of_t, vq_of_t, vvq_of_t = trajs

    q_target = q_of_t(tcurrent)
    v_target = vq_of_t(tcurrent)

    e_d = v_target - vq
    e = q_target - q


    grasp_torque = np.zeros(15)
    grasp_torque[3] = -110
    grasp_torque[9] = 110

    torques = Kp * e + Kd * e_d + grasp_torque

    sim.step(torques)

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    # Setting initial configuration
    sim.setqsim(q0)
    
    def maketraj(q0,q1,T): 
        pointlist = [q0, q0]
        pointlist += path
        pointlist += [q1, q1]
        q_of_t = Bezier(pointlist,t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t

    total_time=4.
    trajs = maketraj(q0, qe, total_time)   
    
    tcur = 0.
    
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
    
    