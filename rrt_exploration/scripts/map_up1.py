#!/usr/bin/env python3.8

import rospy
import time
from nav_msgs.msg import OccupancyGrid
import pandas as pd

file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/baseline_results.csv'

rospy.init_node('delay_calculate', anonymous=True)

previous_time1 = None
previous_time2 = None
previous_time3 = None


def append_to_column(file_path, column_name, new_value):
    df = pd.read_csv(file_path)
    
    if column_name not in df.columns:
        raise ValueError(f"Column '{column_name}' not found in CSV.")
    
    column_data = df[column_name]
    empty_index = column_data[column_data.isna()].index.min()
    
    if pd.isna(empty_index):
        empty_index = len(df)
    
    if empty_index >= len(df):
        df = df.reindex(range(empty_index + 1))
    
    df.loc[empty_index, column_name] = new_value
    
    df.to_csv(file_path, index=False)



def map_callback1(data):
    global previous_time1  
    current_time = data.header.stamp

    if previous_time1 is not None:
        time_difference1 = (current_time - previous_time1).to_sec()

        column_name = 'map_1'  
        new_value = str(time_difference1)
        append_to_column(file_path, column_name, new_value)

    column_name = 'server_rcv1'  
    new_value = str(time.time())
    append_to_column(file_path, column_name, new_value)

    previous_time1 = current_time

def map_callback2(data):
    global previous_time2  
    current_time = data.header.stamp

    if previous_time2 is not None:
        time_difference2 = (current_time - previous_time2).to_sec()

        column_name = 'map_2'  
        new_value = str(time_difference2)
        append_to_column(file_path, column_name, new_value)

    column_name = 'server_rcv2'  
    new_value = str(time.time())
    append_to_column(file_path, column_name, new_value)

    previous_time2 = current_time


def map_callback3(data):
    global previous_time3  
    current_time = data.header.stamp

    if previous_time3 is not None:
        time_difference3 = (current_time - previous_time3).to_sec()

        column_name = 'global'  
        new_value = str(time_difference3)
        append_to_column(file_path, column_name, new_value)

    previous_time3 = current_time



rospy.Subscriber("/jb_0/map", OccupancyGrid, map_callback1)
rospy.Subscriber("/jb_1/map", OccupancyGrid, map_callback2)
rospy.Subscriber("/map", OccupancyGrid, map_callback3)


# rospy.loginfo("Node initialized. Waiting for map messages...")

rospy.spin()