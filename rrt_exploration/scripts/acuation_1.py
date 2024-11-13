#!/usr/bin/env python3.8
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
from std_msgs.msg import Header

goal_start_time1 = None
goal_end_time1 = None

goal_start_time2 = None
goal_end_time2 = None


file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/baseline_results.csv'


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
    


def status_callback1(msg):
    global goal_start_time1, goal_end_time1
    
    for status in msg.status_list:
        if status.status == 1:  # ACTIVE
            if goal_start_time1 is None:
                goal_start_time1 = rospy.Time.now()
                rospy.loginfo(f"Goal is active. Start time: {goal_start_time1}")
        
        elif status.status in [3, 4]:  # SUCCEEDED or ABORTED
            if goal_start_time1 is not None:
                goal_end_time1 = rospy.Time.now()
                actuation_time = goal_end_time1 - goal_start_time1
                rospy.loginfo(f"Goal completed with status: {status.status}. Actuation time: {actuation_time.to_sec()} seconds")

                column_name = 'rbt1_acuation'  
                new_value = str(actuation_time.to_sec())
                append_to_column(file_path, column_name, new_value)

                goal_start_time1 = None  # Reset for the next goal
                goal_end_time1 = None

def status_callback2(msg):
    global goal_start_time2, goal_end_time2
    
    for status in msg.status_list:
        if status.status == 1:  # ACTIVE
            if goal_start_time2 is None:
                goal_start_time2 = rospy.Time.now()
                rospy.loginfo(f"Goal is active. Start time: {goal_start_time2}")
        
        elif status.status in [3, 4]:  # SUCCEEDED or ABORTED
            if goal_start_time2 is not None:
                goal_end_time2 = rospy.Time.now()
                actuation_time = goal_end_time2 - goal_start_time2
                rospy.loginfo(f"Goal completed with status: {status.status}. Actuation time: {actuation_time.to_sec()} seconds")

                column_name = 'rbt2_acuation'  
                new_value = str(actuation_time.to_sec())
                append_to_column(file_path, column_name, new_value)

                goal_start_time2 = None  # Reset for the next goal
                goal_end_time2 = None


def listener():
    rospy.init_node('goal_status_listener', anonymous=True)
    topic1 = '/jb_0/move_base/status'
    topic2 = '/jb_1/move_base/status'

    rospy.Subscriber(topic1, GoalStatusArray, status_callback1)
    rospy.Subscriber(topic2, GoalStatusArray, status_callback2)

    rospy.spin()

if __name__ == '__main__':
    listener()