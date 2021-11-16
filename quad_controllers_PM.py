#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 06:37:23 2021

@author: raduefb
"""
import numpy as np
import pyomo.environ as pyo
import mosek
from quad_trajectory_PM import *
from quad_properties_PM import *


def pos_loop_MPC(quadrotor, N_MPC, k_cur, freq, x0):
    from pyomo.opt import SolverStatus, TerminationCondition
    # quadrotor: quad object of type quad_class
    # N_MPC: horizon of MPC controller
    # k_cur: starting time step
    # freq: frequency controller is run at
    # x0: initial condition of current optimization problem
    
    # Initialize optimization problem
    model = pyo.ConcreteModel()
    model.N = N_MPC
    model.TS = 1/freq
    model.nx = np.size(quadrotor.A, 0)
    model.nu = np.size(quadrotor.B, 1)
    
    # Length of finite optimization problem
    model.tIDX = pyo.Set( initialize= range(model.N+1), ordered=True )  
    model.xIDX = pyo.Set( initialize= range(model.nx), ordered=True )
    model.uIDX = pyo.Set( initialize= range(model.nu), ordered=True )
    
    # Create pyomo objects for the linear system description
    model.A = quadrotor.A
    model.B = quadrotor.B

    # Create state and input variables trajectory
    model.x = pyo.Var(model.xIDX, model.tIDX)
    model.u = pyo.Var(model.uIDX, model.tIDX)
    
    # Import tracking trajectory for current time and horizon
    stages = np.linspace(k_cur, k_cur+N_MPC-1, N_MPC)
    ref_traj = circular_traj(stages, freq)
    
    # Objective
    def stage_cost(model):
        costX = 0.0
        costTerminal = 0.0
        # For all time steps
        for t in model.tIDX:
            # For the three states of interest: x,y,z
            for i in list(range(3,6)):
                if t < model.N-1:
                    costX += (model.x[i, t] - ref_traj[i-3,t])**2
        for i in list(range(3,6)): 
            costTerminal += 4 * (model.x[i, model.N-1] - ref_traj[i-3, model.N-1])**2
        return costX + costTerminal
    model.cost = pyo.Objective(rule = stage_cost, sense = pyo.minimize)
    
    # System Constraints
    def equality_const_rule(model, i, t):
        return  model.x[i, t+1] - (model.x [i, t] + model.TS * (sum(model.A[i, j] * model.x[j, t] for j in model.xIDX)
                +sum(model.B[i, j] * model.u[j, t] for j in model.uIDX))) == 0 if t < model.N else pyo.Constraint.Skip
    model.equality_constraints = pyo.Constraint(model.xIDX, model.tIDX, rule=equality_const_rule)
    
    # Initial Conditions Constraints
    model.initial_constraints = pyo.Constraint(model.xIDX, rule=lambda model,i: model.x[i,0]==x0[i])
    
    # Input Constraints
    model.input_constraints1 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]<=quadrotor.uU[i])
    model.input_constraints2 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]>=quadrotor.uL[i])

    # Initialize MOSEK solver and solve optimization problem
    solver = pyo.SolverFactory("mosek")
    results = solver.solve(model)
    
    # Check if solver found a feasible, bounded, optimal solution
    if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
        feas = True
        xOpt = np.asarray([[model.x[i,t]() for i in model.xIDX] for t in model.tIDX]).T
        uOpt = np.asarray([model.u[:,t]() for t in model.tIDX]).T
        print("MPC problem ", k_cur, " solved!")
    else:
        feas = False
        xOpt = 999
        uOpt = 999

      
    return [feas, xOpt, uOpt]
