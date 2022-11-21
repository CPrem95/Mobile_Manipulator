import rospy
import time
import robotmove as rm
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    try:
        rospy.init_node('to_goal', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/gazebo/model_states"
        state_subscriber = rospy.Subscriber(position_topic, ModelStates, rm.stateCallback) 
        time.sleep(2)

        # goes backward from the goal
        rm.move(velocity_publisher, 0.25, 1.5, False)       #move(velocity_publisher, speed, distance, is_forward)
        # rotates 150 deg ccw
        rm.rel_rotate(velocity_publisher, 30, 150, False)   #rel_rotate(velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise)
        # goes to origin
        rm.go_to_xy(velocity_publisher, 0, 0, 6)            #go_to_xy(velocity_publisher, x_goal, y_goal, K_angular)
        # rotates 90 deg cw
        rm.rel_rotate(velocity_publisher, 30, 90, True)
        # aligns with the world coord frame
        rm.abs_rotate(velocity_publisher, 30, 90)           #abs_rotate(velocity_publisher, angular_speed_degree, angle_degree)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")