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
        
        # The robot do not go straight to the goal
        # the skid steering tends to deviate from the correct position when rotating
        # aThat is the reason for having a combination as below >> move >> rotate loop
        rm.go_to_xy(velocity_publisher, 2.0, 3.0, 6)    #go_to_xy(velocity_publisher, x_goal, y_goal, K_angular)
        rm.abs_rotate(velocity_publisher, 30, 90)       #abs_rotate(velocity_publisher, angular_speed_degree, angle_degree)
        rm.go_to_xy(velocity_publisher, 2.0, 4, 8)
        rm.abs_rotate(velocity_publisher, 30, 90)
        rm.go_to_xy(velocity_publisher, 2.0, 4.52, 6)   #4.5 + 0.02 to compensate the 0.025 positional accuracy threshold
        rm.abs_rotate(velocity_publisher, 30, 90)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")