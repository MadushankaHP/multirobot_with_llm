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
from geometry_msgs.msg import PoseStamped


location = None
direction = None
robot1_status = None
robot2_status = None
dir = None


start_time1 = None
start_time2 = None

file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'


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



def callback1(msg):
    global start_time1
    start_time1 = time.time()
    rospy.loginfo(f"Robot 1 goal received: {msg}")

def callback2(msg):
    global start_time2
    start_time2 = time.time()
    rospy.loginfo(f"Robot 2 goal received: {msg}")

def callback3(msg):
    global robot1_status
    if(msg.status_list != []):
        robot1_status = msg.status_list[0].status
        rospy.loginfo(f"Robot 1 status: {robot1_status}")

def callback4(msg):
    global robot2_status
    if(msg.status_list != []):
        robot2_status = msg.status_list[0].status
        rospy.loginfo(f"Robot 2 status: {robot2_status}")





def main():
    global robot1_status,robot2_status,start_time2,start_time1
    global direction
    

    topic_status_1 = '/jb_0/move_base/status'
    topic_status_2 = '/jb_1/move_base/status'
    topic_goal_1 = '/jb_0/move_base_simple/goal'
    topic_goal_2 = '/jb_1/move_base_simple/goal'
    
    rospy.init_node('action_save_data', anonymous=True)
    
    # Subscribe to robot status topics
    rospy.Subscriber(topic_status_1, GoalStatusArray, callback3)
    rospy.Subscriber(topic_status_2, GoalStatusArray, callback4)
    
    # Subscribe to goal topics (using PoseStamped)
    rospy.Subscriber(topic_goal_1, PoseStamped, callback1)
    rospy.Subscriber(topic_goal_2, PoseStamped, callback2)


    while not rospy.is_shutdown():
        global robot1_status
        global robot2_status
        global dir
        

        if(robot1_status==3 and start_time1 is not None):

            column_name = 'robot1_actuation'  
            new_value = str(time.time()-start_time1)
            #append_to_column(file_path, column_name, new_value)
            start_time1 = None
    
            core= True



        if(robot2_status==3 and start_time2 is not None):

            column_name = 'robot2_actuation'  
            new_value = str(time.time()-start_time2)
            #append_to_column(file_path, column_name, new_value)
            start_time2 = None
            core2= True





        # if(robot2_status!=3):
        #     start_time2 = time.time()

        # elif(robot2_status==3):

        #     end_time2 = time.time()
        #     elapsed_time2 = end_time2 - start_time2
        #     print("----------------")
        #     print(elapsed_time2)
        #     print("----------------")

        

if __name__ == '__main__':
    
    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;