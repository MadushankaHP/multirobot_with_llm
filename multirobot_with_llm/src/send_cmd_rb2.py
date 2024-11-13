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
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

location = None
direction = None
robot1_status = None
dir = None

invalid_points = [
    (-10,8), (-10,6), (-10,4), (-10,2), (-10,0), (-10,-2), (-10,-4), (-10,-6), (-10,-8), (-10,-10),
    (-8,8), (-8,0), (-8,-10), (-6,8), (-6,-10), (-4,8), (-4,0), (-4,-10),
    (-2,8), (-2,6), (-2,4), (-2,2), (-2,0), (-2,-4), (-2,-6), (-2,-8), (-2,-10),
    (0,6), (0,4), (0,2), (0,0), (0,-4), (0,-10), (2,6), (2,4), (2,2), (2,0), (2,-4), (2,-8), (2,-10),
    (4,6), (4,4), (4,2), (4,0), (4,-10), (8,6), (8,4), (8,2), (8,0), (8,-10),
    (10,8), (10,6), (10,4), (10,2), (10,0), (10,-2), (10,-8), (10,-10),
    (12,8), (12,2), (12,-10), (14,8), (14,-10), (16,8), (16,2), (16,-10),
    (18,8), (18,6), (18,4), (18,2), (18,0), (18,-2), (18,-4), (18,-6), (18,-8), (18,-10)
]

file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'

class MoveBaseClient:
    def __init__(self, robot_name):
        # Initialize the action client
        self.client = actionlib.SimpleActionClient(robot_name + '/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

    def send_goal(self, position, orientation):
        # Create a goal to send to the move_base server
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # or "odom", depending on your use case
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position (x, y, z) and orientation (as a quaternion)
        goal.target_pose.pose = Pose(
            Point(position['x'], position['y'], position['z']),
            Quaternion(orientation['x'], orientation['y'], orientation['z'], orientation['w'])
        )

        # Send the goal
        rospy.loginfo("Sending goal...")
        self.client.send_goal(goal)

        # Wait for the result
        wait = self.client.wait_for_result()
        if wait:
            rospy.loginfo("Goal reached!")
        else:
            rospy.logwarn("Action server did not complete the goal!")
current_goal = PoseStamped()

def callback1(message1: Odometry):
    global location
    
    location = message1.pose.pose
    
def callback2(message1: String):
    global direction
    
    direction = message1.data

    
    
# def callback3(message1):
#     global robot1_status
    
#     if(len(message1.status_list)>2):
#         robot1_status = message1.status_list.status
#         print(robot1_status)
    
    
def callback3(msg):
    global robot1_status
    if(msg.status_list !=[]):
        robot1_status = msg.status_list[0].status    
    
def callback4(msg):
    global dir
    dir = msg.data


def save_data(file_path,data):

    with open(file_path, 'r') as file:
        content = file.readlines()

    for line in content:
        print(line.strip())
    data = data+"\n"

    with open(file_path, 'a') as file:
        file.write(data) 

    

def get_direction_coordinate(center, radius, dir,direction,robot_loc):
    # Center coordinates
    closest_direction = None
    min_dif = float('inf')
    x = robot_loc[0]
    y = robot_loc[1]
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
    
    DIRECTIONS = {
            0: (1, 0),    # Right
            45: (1, 1),   # Diagonal up-right
            90: (0, 1),   # Up
            135: (-1, 1), # Diagonal up-left
            180: (-1, 0), # Left
            225: (-1, -1),# Diagonal down-left
            270: (0, -1), # Down
            315: (1, -1)  # Diagonal down-right
        }
    
    direc = dir.split(",")
    print(direc)
    direction = direction.strip().lower()
    
    if(len(direc)>1):
        angle_rad2 = math.radians(directions[direction])
        angle_deg2 = math.degrees(angle_rad2)

        if angle_deg2 < 0:
            angle_deg2 += 360
        
        for i in range(len(direc)):
            angle_rad1 = math.radians(directions[direc[i]])
            angle_deg1 = math.degrees(angle_rad1)

            if angle_deg1 < 0:
                angle_deg1 += 360
                
            dif = abs(angle_deg2-angle_deg1)

            dif = min(dif, 360 - dif)
        
            if dif < min_dif:
                min_dif = dif
                closest_direction = direc[i]
    
    elif(len(direc)==1):
        closest_direction = direc[0]
        
    print("**********")
    print(closest_direction)
    print("**********")
    
    if closest_direction not in directions:
        raise ValueError(f"Invalid direction '{direction}'. Valid directions are: {list(directions.keys())}")
    
    angle_rad = math.radians(directions[closest_direction])
    angle_deg2 = math.degrees(angle_rad)
    angle_deg2 = angle_deg2 % 360

    dx1, dy1 = DIRECTIONS[angle_deg2]
   
    return (x + (dx1*radius), y + (dy1*radius)),closest_direction

def append_to_column(file_path, column_name, new_value):
    # Load the CSV into a DataFrame
    df = pd.read_csv(file_path)
    
    # Check if the column exists
    if column_name not in df.columns:
        raise ValueError(f"Column '{column_name}' not found in CSV.")
    
    # Find the first empty cell in the specified column
    column_data = df[column_name]
    empty_index = column_data[column_data.isna()].index.min()
    
    # If no empty cell, append at the bottom
    if pd.isna(empty_index):
        empty_index = len(df)
    
    # Ensure the DataFrame is large enough to accommodate the new value
    if empty_index >= len(df):
        df = df.reindex(range(empty_index + 1))
    
    # Update the value in the specified column
    df.loc[empty_index, column_name] = new_value
    
    # Save the updated DataFrame back to the CSV
    df.to_csv(file_path, index=False)
    #print(f"Added new value '{new_value}' to column '{column_name}' at row {empty_index}.")

def check_valid_point(invalid_point,point):
    point = tuple(point)
    if point in invalid_point:
        return True #print(f"Point {point_to_check} is valid.")
    else:
        return False #print(f"Point {point_to_check} is invalid.")



def main():
    global location
    global robot1_status
    global direction
    
    topic1 = '/jb_1/move_base_simple/goal';
    topic2 = '/jb_1/odom';
    topic3 = '/jb_1/direction'
    topic4 = '/jb_1/move_base/status'

    # directions = np.array(['east','northeast','north','northwest','west','southwest','south','southeast'])
    
    rospy.init_node('action_node_robot_2',anonymous=True);

    rospy.Subscriber(topic2,Odometry, callback1);
    rospy.Subscriber(topic3,String, callback2);
    rospy.Subscriber(topic4,GoalStatusArray, callback3);
    rospy.Subscriber('/rb2_move_dir',String, callback4);
    
    
    pub1 = rospy.Publisher(topic1, PoseStamped, queue_size=10)
    
    
    core = True
    number =True
    rbt1_currnt_loc = [0,-8]
    rbt2_currnt_loc = (0,-8)
    robot_new_target = [0,-8]
    while not rospy.is_shutdown():
        global robot1_status
        global dir
        if location is not None and direction is not None:
            

            column_name = 'rbt2_rcv'  
            new_value = str(time.time())
            #append_to_column(file_path, column_name, new_value)

                
            
            if(core==True):
                # print("first")
                position = location.position
                orientation = location.orientation
                
                center = (position.x, position.y)
                radius = 2 
                flag2 = False
                while(flag2==False):
                    coord,closed_dir = get_direction_coordinate(center, radius, dir,direction,rbt1_currnt_loc)
                    print("robot 2 check coor:" , coord)
                    flag1 = check_valid_point(invalid_points,coord)
                    print("robot 2  flag:", flag1)
                    
                    if(flag1==True):
                        dir=dir.split(',')
                        if closed_dir in dir:
                            dir.remove(closed_dir)

                        dir = ','.join(dir)

                    else:
                        flag2 = True


                
                flag1 = check_valid_point(invalid_points,coord)
                print("robot 2  flag new:", flag1)
                if(flag1==True):
                    robot_new_target[0] = rbt1_currnt_loc[0]
                    robot_new_target[1] = rbt1_currnt_loc[1]

                else:
                    robot_new_target[0] = coord[0]
                    robot_new_target[1] = coord[1]

                print("robot 2 new loc:", robot_new_target)
                current_goal.header.frame_id = 'map'
                current_goal.pose.position.x = robot_new_target[0]
                current_goal.pose.position.y = robot_new_target[1]
                current_goal.pose.position.z = position.z
                
                current_goal.pose.orientation.x = 0
                current_goal.pose.orientation.y = 0
                current_goal.pose.orientation.z = 0
                current_goal.pose.orientation.w = 1

        #         client = MoveBaseClient("jb_1")  # Replace "robot1" with your robot's name

        # # Define the position and orientation for the goal
        #         position = {'x': robot_new_target[0], 'y': robot_new_target[1], 'z': 0.0}  # Example position
        #         orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}  # Example orientation (facing forward)

        # Send the goal to move the robot
        


                # column_name = 'robot1_actuation_start'  
                # new_value = str(time.time())
                # append_to_column(file_path, column_name, new_value)
                
                print("send robot 2 goal")
                for i in range(20):
                    #client.send_goal(position, orientation)
                    pub1.publish(current_goal)

                column_name = 'rbt2_actuation_strt'  
                new_value = str(time.time())
                #append_to_column(file_path, column_name, new_value)


                core = False
                rbt1_currnt_loc[0] = robot_new_target[0]
                rbt1_currnt_loc[1] = robot_new_target[1]
                
            
            
            
            if(robot1_status==3 and core ==False):
                # print(robot1_status)
                position = location.position
                orientation = location.orientation
                
                center = (position.x, position.y)
                
                radius = 2 

                flag2 = False
                while(flag2==False):
                    coord,closed_dir = get_direction_coordinate(center, radius, dir,direction,rbt1_currnt_loc)
                    print("robot 2 check coor:" , coord)
                    flag1 = check_valid_point(invalid_points,coord)

                    if(flag1==True):
                        dir=dir.split(',')
                        if closed_dir in dir:
                            dir.remove(closed_dir)

                        dir = ','.join(dir)

                    else:
                        flag2 = True

                flag1 = check_valid_point(invalid_points,coord)
                print("robot 2  flag:", flag1)
                if(flag1==True):
                    robot_new_target[0] = rbt1_currnt_loc[0]
                    robot_new_target[1] = rbt1_currnt_loc[1]

                else:
                    robot_new_target[0] = coord[0]
                    robot_new_target[1] = coord[1]
                print("robot 2 new loc:", robot_new_target)
                current_goal.header.frame_id = 'map'
                current_goal.pose.position.x = robot_new_target[0]
                current_goal.pose.position.y = robot_new_target[1]
                current_goal.pose.position.z = position.z
                
                current_goal.pose.orientation.x = 0
                current_goal.pose.orientation.y = 0
                current_goal.pose.orientation.z = 0
                current_goal.pose.orientation.w = 1
               
        #         client = MoveBaseClient("jb_1")  # Replace "robot1" with your robot's name

        # # Define the position and orientation for the goal
        #         position = {'x': robot_new_target[0], 'y': robot_new_target[1], 'z': 0.0}  # Example position
        #         orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}  # Example orientation (facing forward)
                
                print("send robot 2 goal")
                for i in range(20):
                    pub1.publish(current_goal)
                    # client.send_goal(position, orientation)

                column_name = 'rbt2_actuation_strt'  
                new_value = str(time.time())
                #append_to_column(file_path, column_name, new_value)

                rbt1_currnt_loc[0] = robot_new_target[0]
                rbt1_currnt_loc[1] = robot_new_target[1]
                    
            direction = None
                
            
            
            
            
            
        # else:
        #     print("Location not yet initialized.")
        

if __name__ == '__main__':
    
    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;