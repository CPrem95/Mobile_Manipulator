# -*- coding: utf-8 -*-
"""
Created on Sat April 19 16:51:48 2022

@author: Charith Premachandra
"""
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import time
import matplotlib.pyplot as plt
import sys

T = int(sys.argv[1]) # time to record in seconds
sampT = 3            # sampling time (between two data points) in seconds

# position coordinates 
global x, y, t0
x = []
y = []
t0 = 0

# solve matrix Ax = B using Gauss elimination
def solveMat(A, B):
    # print('Matrix [A]:')
    # print(A)
    # print('\nMatrix [B]:')
    # print(B)
    
    r,c = A.shape
    
    # Forward elimination
    for i in range(r -1):           # loop over all rows in matrix, except last
        for j in range(i +1, r):    # loop over all rows below diagonal
            cons = A[j,i]/A[i,i]    # compute multiplier
            A[j] = A[j] - A[i]*cons # update row i and column j
            B[j] = B[j] - B[i]*cons # update right-hand side row i and column j
    
    
    Sol = np.zeros(r)               # initialize solution
    
    # Compute last unknown
    Sol[r -1] = (float(B[r -1]/A[r -1, r -1]))
    
    # Back Substitution
    for i in range(r -2, -1, -1):# loop backwards over all rows except last
        Sum = 0
        for j in range(i +1, c): # loop over all columns to the right of the current row
            Sum += A[i, j]*Sol[j]
        
        Sol[i] = (B[i] - Sum)/A[i, i]   # compute i th unknown 
        
    print('\nSolutions:', Sol)
    return (Sol)

# callback func to constantly update the odometry readings with a sample time of sampT
def plotCallback(odom_message):
    global x, y, t0
    if t0 == 0:
        t0 = rospy.Time.now().to_sec()

    x.append(odom_message.pose.pose.position.x * 100)
    y.append(odom_message.pose.pose.position.y * 100)
    
    rospy.loginfo(rospy.get_caller_id() + ' odometry received')

    if rospy.Time.now().to_sec() - t0 > T:
        rospy.signal_shutdown("Finish") 
    else:
        time.sleep(sampT)

# QUADRATIC splines plotting function
# inputs:: x and y coordinates of a points list separately x = [...], y = [...]
def plotPath(x, y):
    plt.figure()
    plt.scatter(x, y, c = 'g')
    plt.plot(x, y, 'b')

    # This valids only for the first three points 
    # They are different from the others since first segment: a0 = 0 and the end needs the next func segment's derivative. 
    # Used the derivation in variables to generate the following matrices
    A = np.matrix([[1,0,0,0,0,0], [x[0]**2,x[0],1,0,0,0], [x[1]**2,x[1],1,0,0,0], [0,0,0,x[2]**2,x[2],1], [2*x[1],1,0,-2*x[1],-1,0], [0,0,0,x[1]**2,x[1],1]]).astype(np.float32)
    B = np.matrix([[0], [y[0]], [y[1]], [y[2]], [0], [y[1]]]).astype(np.float32)
    sol = solveMat(A, B)
    
    # Plot the first segment
    xp = np.linspace(x[0], x[1], 20)
    yp = sol[0]*xp**2 + sol[1]*xp + sol[2]
    plt.plot(xp, yp, 'r')

    # Plot the second segment
    xp = np.linspace(x[1], x[2], 20)
    yp = sol[3]*xp**2 + sol[4]*xp + sol[5]
    plt.plot(xp, yp, 'r')

    # Iterate through the next segments 
    for i in range(2, len(x)-1):
        A = np.matrix([[x[i]*2, 1, 0], [x[i+1]**2, x[i+1], 1], [x[i]**2, x[i], 1]]).astype(np.float32)
        B = np.matrix([[x[i]*2*sol[-3] + sol[-2]], [y[i+1]], [y[i]]]).astype(np.float32)
        # print(B)
        sol = solveMat(A, B)
        
        # Plotting the next segment
        xp = np.linspace(x[i], x[i+1], 20)
        yp = sol[0]*xp**2 + sol[1]*xp + sol[2]
        plt.plot(xp, yp, 'r')

    plt.axis('equal')
    plt.title('Floor space')
    plt.xlabel('X [cm]')
    plt.ylabel('Y [cm]')
    plt.legend(['Data points', 'Linear splines', 'Quadratic splines'])

    plt.show(block=True)

# initialize node 
def listener():
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odomListener', anonymous=True)

    # callback will update the odometry points
    rospy.Subscriber("/odom", Odometry, plotCallback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
    # print(x)
    # print(y)
    plotPath(x, y)
    print('\-----------------------------------')
    print('Completed')
    print('-----------------------------------')