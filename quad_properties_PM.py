#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 02:38:02 2021

@author: raduefb
"""
import numpy as np


# Create quadrotor class
class quad_class_PM:
    def __init__(self, mass, A, B, uL, uU):
        self.mass = mass # Mass (kg)
        self.A = A # xdot = Ax + Bu
        self.B = B # xdot = Ax + Bu
        self.uL = uL # uL < u < uU
        self.uU = uU
        self.g = 9.81 # (m/s^2)


# Create multiple quadrotor objects
def quad_MATLAB_sim_PM():
    m = 0.85 # (kg)
    
    # xdot = Ax + Bu (linear system description)
    A = np.zeros((6,6))
    A[3,0] = 1; A[4,1] = 1; A[5,2] = 1
    B = np.zeros((6,3))
    B[0,0] = 1/m; B[1,1] = 1/m; B[2,2] = 1/m
    
    # u constraints
    uU = np.array([20, 20, 20])
    uL = np.array([-20, -20, -20])
    return m, A, B, uL, uU