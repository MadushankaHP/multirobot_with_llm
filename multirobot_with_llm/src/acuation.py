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

location = None
direction = None
robot1_status = None
robot2_status = None
dir = None


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



    
    
def callback3(msg):
    global robot1_status
    if(msg.status_list !=[]):
        robot1_status = msg.status_list[0].status    
    
def callback4(msg):
    global robot2_status
    if(msg.status_list !=[]):
        robot2_status = msg.status_list[0].status 


def main():
    global robot1_status
    global direction
    

    topic4 = '/jb_0/move_base/status'
    topic5 = '/jb_1/move_base/status'

    # directions = np.array(['east','northeast','north','northwest','west','southwest','south','southeast'])
    
    rospy.init_node('save_data',anonymous=True);

 
    rospy.Subscriber(topic4,GoalStatusArray, callback3);
    rospy.Subscriber(topic5,GoalStatusArray, callback4);

    
    
    
    
    start_time1 = time.time()
    start_time2 = time.time()
    core = False
    number =True
    core2 = False
    while not rospy.is_shutdown():
        global robot1_status
        global robot2_status
        global dir
        

        if(robot1_status==3 and core==False):

            column_name = 'robot1_actuation_end'  
            new_value = str(time.time())
            append_to_column(file_path, column_name, new_value)

    
            core= True

        elif(robot1_status==1):
            core = False

        if(robot2_status==3 and core2==False):

            column_name = 'robot2_actuation_end'  
            new_value = str(time.time())
            append_to_column(file_path, column_name, new_value)
    
            core2= True

        elif(robot2_status==1):
            core2 = False



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