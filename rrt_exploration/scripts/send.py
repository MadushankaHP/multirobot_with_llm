#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


check_location = False;
traget_location = None;
target_goal = PoseStamped()


    



        



def send_goal(robot_name, target_position):
    rospy.init_node('target_pos_pub',anonymous=True);

    topic2 = "/"+ str(robot_name)+'/move_base_simple/goal';


    pub1 = rospy.Publisher(topic2, PoseStamped,queue_size=10);

    while not rospy.is_shutdown():
        

            target_goal.header.frame_id = str(robot_name)+"/map"
            target_goal.pose.position.x = target_position[0]
            target_goal.pose.position.y = target_position[1]
            target_goal.pose.position.z = 0
            target_goal.pose.orientation.x = 0
            target_goal.pose.orientation.y = 0
            target_goal.pose.orientation.z = 0
            target_goal.pose.orientation.w = 1

            time.sleep(3)
            pub1.publish(target_goal)
            print("Successfully published target location"+"("+traget_location+")")
            
if __name__ == '__main__':

    try:
        send_goal();
    except:
        rospy.logerr('Subscriber is error');
        pass;