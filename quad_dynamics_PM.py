#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 13:57:39 2021

@author: raduefb
"""

import numpy as np
from scipy.integrate import solve_ivp
from quad_properties_PM import quad_MATLAB_sim_PM


def quad_DE_PM(t, a, fx, fy, fz, quadrotor):
    # Unpack States
    u, v, w, x, y, z = a

    # NOT TRUE FOR POINT MASS SIMULATION
    # Create storage for RHS of DE
    # DE is taken from F. Sabatino, "Quadrotor Control: modeling, nonlinear
    # control design, and simulation", June 2015, KTH Sweden
    state = np.empty(6)

    state[0] = fx / quadrotor.mass
    state[1] = fy / quadrotor.mass
    state[2] = fz / quadrotor.mass
    state[3] = u
    state[4] = v
    state[5] = w
    return [state]

def quad_dyn_PM(quadrotor, TS, x_k, u_k):
    # Solve non-linear time-invariant differential equation at the next timestep
    sol = solve_ivp(fun=quad_DE_PM, t_span=[0, TS], y0=x_k, method='RK45', 
                    t_eval = [TS], vectorized=True, args=(u_k[0], u_k[1], u_k[2], quadrotor), rtol=1e-7)
    # Append states at x_k+1 to return vector
    x_k1 = sol.y
    return x_k1


