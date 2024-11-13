import pandas as pd

columns = [
    'robot1_actuation', 'robot2_actuation', 'robot1_send_text', 'robot2_send_text.1', 
    'Server_recive_text1', 'Server_recive_text2', 'RRT_start', 'RRT_finish', 
    'server_2_robot1', 'server_2_robot2', 'robot1_recived_dir', 'robot2_recived_dir'
]

# Create an empty DataFrame with these columns
df = pd.DataFrame(columns=columns)
df.to_csv('/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv', index=False)