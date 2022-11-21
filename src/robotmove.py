#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
import math
import time
from std_srvs.srv import Empty
import tf

# T ransforming (rotation) of the base coordinate frame of the robot to align with the global coord frame
origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
qz = tf.transformations.quaternion_about_axis(-math.pi/2, zaxis)
# Rz = tf.transformations.rotation_matrix(math.pi/2, zaxis)

# This function continuosly updates the global state variables (position and velocities)
def stateCallback(state_message): 
    global x, y
    global yaw
    global vx, vy
    global omyaw

    name = 2

    x = state_message.pose[name].position.x
    y = state_message.pose[name].position.y

    q = (
    state_message.pose[name].orientation.x,
    state_message.pose[name].orientation.y,
    state_message.pose[name].orientation.z,
    state_message.pose[name].orientation.w)

    q = tf.transformations.quaternion_multiply(q, qz)

    euler = tf.transformations.euler_from_quaternion(q)
    # roll = euler[0]
    # pitch = euler[1]
    # yaw = euler[2] + math.pi
    yaw = euler[2] 

    # print([x, y, yaw])

    vx = state_message.twist[name].linear.x
    vy = state_message.twist[name].linear.y

    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3
    omyaw = state_message.twist[name].angular.z

# moving the robot in a straight line
def move(velocity_publisher, speed, distance, is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()
    #get current location 
    global x, y
    x0 = x
    y0 = y

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
        
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # publish the velocity at 10 Hz (10 times a second)    
    
    while distance_moved < distance:
            rospy.loginfo("Robot moves")
            velocity_publisher.publish(velocity_message)

            loop_rate.sleep()
            
            distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
            print('distance_moved:', distance_moved)

    rospy.loginfo("reached")
    
    #finally, stop the robot when the distance is moved
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)

# Rotates the robot with respect to the world coordinate frame
def abs_rotate(velocity_publisher, angular_speed_degree, angle_degree):
    global yaw
    yaw0 = yaw
    
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))

    loop_rate = rospy.Rate(30) # publish the velocity at 10 Hz    

    # PID parameters
    P = 1
    I = 0.02
    D = 0.1
    angle_rad = math.radians(angle_degree)  # absolute angle relative to world coordinates system
    relative_angle = abs(angle_rad - yaw0)  # calculate the relative angle with resp to the current yaw
    error = angle_rad - yaw                 # error 
    sum = 0                         # for the integral term of PID
    prev_err = 0                    # error calculation of PID

    thresh = math.radians(0.2)
    while abs(error) > thresh:
        rospy.loginfo("Robot rotates")
        error = angle_rad - yaw

        # integration starts only when the angle to go is low 
        if abs(error) < math.radians(15):
            sum = sum + error

        # Reset the sum
        if sum > 150:
            sum = 0

        if (relative_angle < math.pi):
            velocity_message.angular.z = -1*error*angular_speed*P - sum*I + (error - prev_err)*D
        else:
            velocity_message.angular.z = error*angular_speed*P + sum*I - (error - prev_err)*D
        
        prev_err = error

        velocity_publisher.publish(velocity_message)
        print('Robot yaw', math.degrees(yaw), 'error', math.degrees(error))
        # t1 = rospy.Time.now().to_sec()
        # current_angle_degree = (t1-t0)*angular_speed_degree
        # print(math.degrees(abs(yaw - yaw0)))
        loop_rate.sleep()

    #finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)
    print("\n-----------------------------------------")
    print("REACHED Orientation")
    print("-----------------------------------------")

# Rotates the robot in a relative angle (from present orientation)
def rel_rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    global omyaw # robot's angular speed

    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))

    angle_moved = 0.0
    loop_rate = rospy.Rate(30) # we publish the velocity at 10 Hz   

    if (clockwise):
        velocity_message.angular.z = abs(angular_speed)
    else:
        velocity_message.angular.z = -abs(angular_speed)

    t0 = rospy.Time.now().to_sec() # This method uses the angular speed and the rotation time. Here takes time 0
    current_angle_degree = 0
    while abs(current_angle_degree) < relative_angle_degree:

        rospy.loginfo("Robot rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree += (t1-t0)*math.degrees(omyaw) # current angle calculation (rotated angle up to time t1) 
        t0 = t1
        loop_rate.sleep()
        print('Robot rotated:', current_angle_degree)
            
    rospy.loginfo("reached")

    #finally, stop the robot when the distance is movedf
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

# This func will drive the robot to a given (x, y) goal 
# K_angular is crucial to stop the robot with sufficient orientation accuracy
# which indirectly affects the positional accuracy.
# K_angular needs to be adjusted accordingly (depending on the distance to the goal)
def go_to_xy(velocity_publisher, x_goal, y_goal, K_angular):
    global x, y
    global yaw

    velocity_message = Twist()

    K_linear = 0.45 # linear speed control parameter
    # This value can be brought to the func def for more complex and precise control
    
    while (True):
        rospy.loginfo("Robot moves")
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear # speed becomes 0 when the robot reaches the goal
        if linear_speed > 0.22:
            linear_speed = 0.22

        desired_angle_goal = math.atan2(y_goal-y, x_goal-x) # the robot stops at the goal with this angle
        angular_speed = (desired_angle_goal-yaw)*K_angular
        # print('desired angle:', math.degrees(desired_angle_goal), 'yaw:', math.degrees(yaw))
        if angular_speed > 1.5:
            angular_speed = 1.5
        elif angular_speed < -1.5:
            angular_speed = -1.5

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = -angular_speed
        # print(['lin sp:', linear_speed, 'ang sp:', angular_speed])
        velocity_publisher.publish(velocity_message)
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        # accuracy 0.025 m
        # This value can be brought to the func def for more complex and precise control
        if (distance < 0.025):
            velocity_message.linear.x = 0
            velocity_message.angular.z = 0
            velocity_publisher.publish(velocity_message)
            print("\n-----------------------------------------")
            print("REACHED Destination")
            print("-----------------------------------------")
            break