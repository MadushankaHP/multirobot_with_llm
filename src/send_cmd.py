#!/usr/bin/env python3
import os
import time
import rospy
import math
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

location = None
direction = None
robot1_status = None

current_goal = PoseStamped()

def callback1(message1: Odometry):
    global location
    
    location = message1.pose.pose
    
def callback2(message1: String):
    global direction
    
    direction = message1.data
    
def callback3(message1):
    global robot1_status
    
    robot1_status = message1.status_list.status
    
    
    
    
    

def get_direction_coordinate(center, radius, direction):
    # Center coordinates
    x, y = center
    directions = {
        'north': 90,
        'northeast': 45,
        'east': 0,
        'southeast': -45,
        'south': -90,
        'southwest': -135,
        'west': 180,
        'northwest': 135
    }
    
    if direction not in directions:
        raise ValueError(f"Invalid direction '{direction}'. Valid directions are: {list(directions.keys())}")
    
    angle_rad = math.radians(directions[direction])
    
    dx = radius * math.cos(angle_rad)
    dy = radius * math.sin(angle_rad)
   
    return (x + dx, y + dy)


def main():
    global location
    global robot1_status
    global direction
    
    topic1 = '/jb_0/move_base_simple/goal';
    topic2 = '/jb_0/odom';
    topic3 = '/robot1_dire'
    topic4 = '/jb_0/move_base/status'

    # directions = np.array(['east','northeast','north','northwest','west','southwest','south','southeast'])
    
    rospy.init_node('action_node_robot_1',anonymous=True);

    rospy.Subscriber(topic2,Odometry, callback1);
    rospy.Subscriber(topic3,String, callback2);
    rospy.Subscriber(topic4,GoalStatusArray, callback3);
    
    
    pub1 = rospy.Publisher(topic1, PoseStamped, queue_size=10)
    
    
    core = True
    while not rospy.is_shutdown():
        if location is not None and direction is not None:
            
            
            
            if(core==True):
                
                position = location.position
                orientation = location.orientation
                
                center = (position.x, position.y)
                radius = 2 
                coord = get_direction_coordinate(center, radius, direction)
                
                current_goal.header.frame_id = 'map'
                current_goal.pose.position.x = coord[0]
                current_goal.pose.position.y = coord[1]
                current_goal.pose.position.z = position.z
                
                current_goal.pose.orientation.x = 0
                current_goal.pose.orientation.y = 0
                current_goal.pose.orientation.z = 0
                current_goal.pose.orientation.w = 1

                print(current_goal.pose.position)
                for i in range(10):
                    pub1.publish(current_goal)
                core = False
                
            
            
            
            if(robot1_status==3 and core ==False):
                print(robot1_status)
                position = location.position
                orientation = location.orientation
                
                center = (position.x, position.y)
                radius = 2 
                coord = get_direction_coordinate(center, radius, direction)
                
                current_goal.header.frame_id = 'map'
                current_goal.pose.position.x = coord[0]
                current_goal.pose.position.y = coord[1]
                current_goal.pose.position.z = position.z
                
                current_goal.pose.orientation.x = 0
                current_goal.pose.orientation.y = 0
                current_goal.pose.orientation.z = 0
                current_goal.pose.orientation.w = 1

                print(current_goal.pose.position)
                for i in range(10):
                    pub1.publish(current_goal)
                
            
            
            
            
            
        else:
            print("Location not yet initialized.")
        

if __name__ == '__main__':
    
    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;