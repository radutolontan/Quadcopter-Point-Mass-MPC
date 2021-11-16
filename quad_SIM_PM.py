#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 08:42:47 2021

@author: raduefb
"""
# Import Python functions
import numpy as np
import matplotlib.pyplot as plt

# Import User functions for the quadcopter
from quad_controllers_PM import pos_loop_MPC
from quad_properties_PM import *
from quad_dynamics_PM import quad_dyn_PM
from quad_trajectory_PM import circular_traj, corner_traj

# Select desired quadrotor type and create its object
m, A, B, uL, uU = quad_MATLAB_sim_PM()
quadrotor = quad_class_PM(m, A, B, uL, uU)
    
# Set operating frequencies for the inner and outer loops
freq_outer = 100 # (Hz)

# Set number of time steps
N = 400
# Set MPC horizon
N_MPC = 15

# Allocate storage for states over time
x = np.empty((6,N+1))

# Set initial conditions on states
x[:,0] = np.array([0,-0.5,0,-1.4,-0.1,0.6]).T


for i in range(0, N):
# --------------------------------------------------------------------------- #
# -------------------------- POSITION CONTROL LOOP -------------------------- #
# --------------------------------------------------------------------------- #
    feas, xMPC, uMPC = pos_loop_MPC(quadrotor, N_MPC, i, freq_outer, x[:,i])
    
    # If at any point the problem returns infeasible, exit the solver
    if not feas:
        x = []
        u = []
        break    
    
# --------------------------------------------------------------------------- #
# ---------------------------- QUADROTOR DYNAMICS --------------------------- #
# --------------------------------------------------------------------------- #
    # Run the system dynamics to obtain the next state
    x_k1 = quad_dyn_PM(quadrotor, 1/freq_outer, x[:,i], uMPC[:,0])
    x[:,i+1] = x_k1.T
    

# Compute Open Loop CFTOC response for comparison
ref_traj = circular_traj(np.linspace(0, N-1, N), freq_outer)
feas, xOL, uOL = pos_loop_MPC(quadrotor, N, 0, freq_outer, x[:,0])

# 2D X-Y plot
fig = plt.figure(1, figsize=(18,6))
plt.subplot(1,3,1)
plt.plot(ref_traj[0,:], ref_traj[1,:], 'green')
plt.plot( xOL[3,:]  ,  xOL[4,:], 'magenta')
plt.plot(  x[3,:]  , x[4,:]    ,'blue')
plt.scatter(x[3,0] , x[4,0], s =15, c='red')
plt.title('X vs. Y')

# 2D X-Z plot
plt.subplot(1,3,2)
plt.plot(ref_traj[0,:], ref_traj[2,:], 'green')
plt.plot( xOL[3,:]  ,  xOL[5,:], 'magenta')
plt.plot(  x[3,:]  , x[5,:]    ,'blue')
plt.scatter(x[3,0] , x[5,0], s =15, c='red')
plt.title('X vs. Z')

# 2D Y-Z plot
plt.subplot(1,3,3)
plt.plot(ref_traj[1,:], ref_traj[2,:], 'green')
plt.plot( xOL[4,:]  ,  xOL[5,:], 'magenta')
plt.plot(  x[4,:]  , x[5,:]    ,'blue')
plt.scatter(x[4,0] , x[5,0], s =15, c='red')
plt.title('Y vs. Z')
plt.legend(['Reference Trajectory','Open Loop (CFTOC)','Closed Loop (MPC)','Initial Condition (x0)'])


# 3D X-Y-Z plot
fig = plt.figure(2, figsize=(8,8))
ax = plt.axes(projection='3d')
plt.plot(ref_traj[0,:], ref_traj[1,:], ref_traj[2,:], 'green')
plt.plot( xOL[3,:]  ,  xOL[4,:],   xOL[5,:], 'magenta')
plt.plot(  x[3,:]  , x[4,:]   , x[5,:]  ,'blue')
plt.xlabel("X")
plt.ylabel("Y")
plt.legend(['Reference Trajectory','Open Loop (CFTOC)','Closed Loop (MPC)'])


    
    



    
    

        
        
    

