#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import time
from scaraKinematics import invKine

#create a new publisher. we specify the topic name, then type of message then the queue size
pub_shoulder = rospy.Publisher('simple_model/shoul_J_position_controller/command', Float64, queue_size=10)
pub_elbow = rospy.Publisher('simple_model/elbow_J_position_controller/command', Float64, queue_size=10)
pub_wrist_R = rospy.Publisher('simple_model/shaft_RJ_position_controller/command', Float64, queue_size=10)
pub_wrist_P = rospy.Publisher('simple_model/shaft_PJ_position_controller/command', Float64, queue_size=10)

#initialize the node - with unique node name
rospy.init_node('invKine', anonymous=True)

def talker(joint_params):
    th_1 = math.radians(joint_params[0])
    th_2 = math.radians(joint_params[1])
    th_3 = math.radians(joint_params[2])    
    
    #set the loop rate
    rate = rospy.Rate(1) # 1hz
    #keep publishing until a Ctrl-C is pressed
    i = 0
    #while not rospy.is_shutdown():
    print("Publishing:")
    rospy.loginfo(th_1)
    pub_shoulder.publish(th_1)
    rospy.loginfo(th_2)
    pub_elbow.publish(th_2)
    rospy.loginfo(th_3)
    pub_wrist_R.publish(th_3)
    rate.sleep()
    rospy.loginfo(joint_params[3]*0.001)
    pub_wrist_P.publish(joint_params[3]*0.001)
    rate.sleep()

if __name__ == '__main__':
    try:
        # Following are waypoints
        # Get an idea how the robot performs the task while reaching the waypoints.
        # Notice that the result of the previous angles (inv kinematics) becomes inputs to find the next angle combination
        # This allows faster convergence

        result = invKine(247.49, -247.49, 190, 0, 0 , -30) #randomly selected, but the start angles were carefully selected
        print("\nWay Point - 1")
        print("-----------------------------------------")
        result = invKine(200, 0, 190, 0, result[0] , result[1])
        talker(result)

        print("\nWay Point - 2")
        print("-----------------------------------------")
        result = invKine(250, 250, 190, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nWay Point - 3")
        print("-----------------------------------------")
        result = invKine(0, 300, 90, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nPICK Position") #(x, y, z) = (0, 300, 40) with resp to the manipulator
        print("-----------------------------------------")
        result = invKine(0, 300, 40, 0, result[0] , result[1]) 
        talker(result)
        time.sleep(3)

        print("\nEnd effector ACTIVATE")
        print("-----------------------------------------")
        time.sleep(3)

        print("\nWay Point - 3")
        print("-----------------------------------------")
        result = invKine(0, 300, 90, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nWay Point - 2")
        print("-----------------------------------------")
        result = invKine(250, 250, 190, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nWay Point - 4")
        print("-----------------------------------------")
        result = invKine(303.11, 0, 240, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nWay Point - 5")
        print("-----------------------------------------")
        result = invKine(303.11, -175, 240, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nPLACE Position") #(x, y, z) = (350sin60, -350cos60, 190) with resp to the manipulator
        print("-----------------------------------------")
        result = invKine(303.11, -175, 190, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nEnd effector DEACTIVATE")
        print("-----------------------------------------")
        time.sleep(3)

        print("\nWay Point - 5")
        print("-----------------------------------------")
        result = invKine(303.11, -175, 240, 0, result[0] , result[1])
        talker(result)
        time.sleep(3)

        print("\nFinal Position - Resting")
        print("-----------------------------------------")
        result = invKine(150, -75, 190, 0, result[0] , result[1])
        talker(result)

        print("\n-----------------------------------------")
        print("COMPLETED")
        print("-----------------------------------------")

    except rospy.ROSInterruptException:
        pass
