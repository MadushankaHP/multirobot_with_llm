#!/usr/bin/env python3.8
import pandas as pd
import rospy


rospy.init_node('create_csv',anonymous=True);


columns = [
    'rbt1_scan','rbt2_scan','rbt1_fov','rbt2_fov','rbt1_2_server','rbt2_2_server','server_rcv1','server_rcv2','map_1','map_2','global','rrt_1','rrt_2','server_robot1',
    'server_robot2','rbt1_rcv','rbt2_rcv','rbt1_acuation','rbt2_acuation'
]

# Create an empty DataFrame with these columns
df = pd.DataFrame(columns=columns)
df.to_csv('/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/baseline_results.csv', index=False)


    