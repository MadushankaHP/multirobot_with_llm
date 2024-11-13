#!/usr/bin/env python3.8

import rospy
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import String
import time
import pandas as pd


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


def feedback_callback1(feedback):
    movement_start_time = str(time.time())
    column_name = 'rbt1_rcv'  
    new_value = str(movement_start_time)
    append_to_column(file_path, column_name, new_value)
    
def feedback_callback2(feedback):
    movement_start_time = str(time.time())
    column_name = 'rbt2_rcv'  
    new_value = str(movement_start_time)
    append_to_column(file_path, column_name, new_value)
    

if __name__ == '__main__':
    rospy.init_node('robot_movements', anonymous=True)


    rospy.Subscriber('/jb_0/move_base/feedback', MoveBaseActionFeedback, feedback_callback1)
    rospy.Subscriber('/jb_1/move_base/feedback', MoveBaseActionFeedback, feedback_callback2)

    rospy.spin()