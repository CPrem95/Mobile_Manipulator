# -*- coding: utf-8 -*-
"""
Created on Sat Mar 26 23:43:44 2022

@author: Charith Premachandra
"""

import numpy as np
import sympy as sp
import math
import matplotlib.pyplot as plt

def invKine(x, y, z, alpha, theta_1, theta_2):
    L1 = 200 # linkage parameters of the FANUC robot
    L2 = 200
    S = 40

    #initial values
    ini_th_1 = math.radians(theta_1)
    ini_th_2 = math.radians(theta_2)
    
    th_1, th_2 = sp.symbols('th_1 th_2')

    # forward kinematics equations (given thetas >>> calculate x, y)
    F1 = L1*sp.cos(th_1) + L2*sp.cos(th_1 + th_2) - x
    F2 = L1*sp.sin(th_1) + L2*sp.sin(th_1 + th_2) - y
    
    # prepare matrices to be substituted in the Newton Raphson method
    th_n = sp.Matrix([[ini_th_1], [ini_th_2]])
    F_n = sp.Matrix([[F1], [F2]])
    J_n = sp.Matrix([[sp.diff(F1, th_1), sp.diff(F1, th_2)], [sp.diff(F2, th_1), sp.diff(F2, th_2)]])
    
    accuracy = 100 # initial accuracy
    thresh = 0.001 # positional accuracy
    n = 1           # iteration number
    acc = []        # to store the accuracy results

    print('calculating...')
    while n < 100 and accuracy > thresh:
        # Newton Raphson method
        J_n = J_n.evalf(subs={th_1: th_n[0], th_2: th_n[1]})
        J_n_inv = J_n.inv()
        th_N = th_n - J_n_inv*F_n

        th_N = th_N.evalf(subs={th_1: th_n[0], th_2: th_n[1]})
        epsilon = th_N - th_n

        posn = F_n.evalf(subs={th_1: th_N[0], th_2: th_N[1]})
        accuracy = posn.norm(1) # find the norm to get accuracy
        acc.append(accuracy)

        th_n = th_N
        n +=1

    print('calculated...')
    print("accuracy:", accuracy)
    th_1_deg = math.degrees(th_n[0])
    th_2_deg = math.degrees(th_n[1])
    th_3_deg = alpha - th_1_deg - th_2_deg
    d = z + S
    print("result:", [th_1_deg, th_2_deg, th_3_deg, d])
    plt.plot(acc)
    plt.title('SCARA Convergence plot')
    plt.ylabel('Positional accuracy [m]')
    plt.xlabel('iteration')

    plt.show(block=True)
    return ([th_1_deg, th_2_deg, th_3_deg, d, accuracy])