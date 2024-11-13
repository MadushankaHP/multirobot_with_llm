#!/usr/bin/env python3

import rospy
import time

def main_loop():
    # Initialize ROS node
    rospy.init_node('loop_time_node', anonymous=True)

    # Set a loop rate (e.g., 10 Hz)
    loop_start_time = time.time()
    # Open a text file to save loop times
    
    while not rospy.is_shutdown():

        loop_end_time = time.time()

        loop_time = loop_end_time - loop_start_time

            # rospy.loginfo(f"Loop Time: {loop_time} seconds")
        with open('/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/base_loop_times.txt', 'w') as file:
            file.write(f"Loop Time: {loop_time} seconds")


if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
