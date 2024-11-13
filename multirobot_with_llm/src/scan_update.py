#!/usr/bin/env python3

import rospy
from std_msgs.msg import String  
import time
import pandas as pd
from sensor_msgs.msg import LaserScan


last_update_time1 = None
current_update_time1 = None

last_update_time2 = None
current_update_time2 = None
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

def callback1(data):
    global last_update_time1, current_update_time1
    
    current_update_time1 = rospy.get_time()
    
    if last_update_time1 is not None:
        time_difference = current_update_time1 - last_update_time1
        # rospy.loginfo(f'Time difference since last update: {time_difference:.2f} seconds')

        column_name = 'rbt1_scan'  
        new_value = str(time_difference)
        append_to_column(file_path, column_name, new_value)
    time.sleep(4)
    last_update_time1 = current_update_time1

def callback2(data):
    global last_update_time2, current_update_time2
    
    current_update_time2 = rospy.get_time()
    
    if last_update_time2 is not None:
        time_difference = current_update_time2 - last_update_time2
        # rospy.loginfo(f'Time difference since last update: {time_difference:.2f} seconds')

        column_name = 'rbt2_scan'  
        new_value = str(time_difference)
        append_to_column(file_path, column_name, new_value)
    time.sleep(4)
    last_update_time2 = current_update_time2

def listener():
    rospy.init_node('scan_update_time', anonymous=True)
    
    rospy.Subscriber('/jb_0/scan', LaserScan, callback1) 
    rospy.Subscriber('/jb_1/scan', LaserScan, callback2)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
