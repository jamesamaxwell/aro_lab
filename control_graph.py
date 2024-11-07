#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import time
import matplotlib.pyplot as plt

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 2500               # proportional gain (P of PD)
Kd = 1   # derivative gain (D of PD)
def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    #TODO 
    torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    print("torques:")
    print(torques)
    
    # Given trajs (position, velocity and acceleration)
    # determine the required torque to move the robot

    q_of_t, vq_of_t, vvq_of_t = trajs

    q_target = q_of_t(tcurrent)
    v_target = vq_of_t(tcurrent)
    a_target = vvq_of_t(tcurrent)

    print("q:", q)
    print("q_target", q_target)

    # Acceleration Calculation
    if tcurrent == 0:
        vq_old = 0

    #vvq = (vq_old - vq) / DT

    e_d = v_target - vq
    e = q_target - q

    grasp_torque = np.zeros(15)
    grasp_torque[3] = -100
    grasp_torque[9] = 100

    print("e:", e)
    print(Kp * e)
    print("Grasp torque", grasp_torque)

    torques = Kp * e + Kd * e_d + grasp_torque

    torques = torques * 1#/2.5
    print("torques:")
    print(torques)
    # time.sleep(3)

    vq_old = vq

    sim.step(torques)

    return q, q_target#np.linalg.norm(q), np.linalg.norm(q_target) 

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    from path_example import get_path
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    # path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    #setting initial configuration
    path = get_path()
    sim.setqsim(q0)
    # print("Path:")
    # print(path)
    
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    def maketraj(q0,q1,T): #TODO compute a real trajectory !
        pointlist = [q0, q0]
        pointlist += path
        pointlist += [q1, q1]
        q_of_t = Bezier(pointlist,t_max=T)
        #q_of_t =    # Make without bezier first
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    
    #TODO this is just a random trajectory, you need to do this yourself
    total_time=4.
    trajs = maketraj(q0, qe, total_time)   
    
    tcur = 0.

    times = []
    
    q_errs = []
    vq_errs = []
    
    while tcur < total_time:
        q_err, vq_err = rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)

        q_errs.append(q_err)
        vq_errs.append(vq_err)
        times.append(tcur)

        tcur += DT

    plt.plot(times, q_errs)
    plt.plot(times,vq_errs)
    #plt.plot(times, vvq_errs)
    plt.xlabel("time (in s)")
    plt.ylabel("error")

    plt.show()   
    
    
    