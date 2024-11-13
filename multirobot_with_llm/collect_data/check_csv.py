import pandas as pd

# def append_to_column(file_path, column_name, new_value):
#     # Load the CSV into a DataFrame
#     df = pd.read_csv(file_path)
    
#     # Check if the column exists
#     if column_name not in df.columns:
#         raise ValueError(f"Column '{column_name}' not found in CSV.")
    
#     # Find the first empty cell in the column
#     empty_index = df[column_name].first_valid_index()
    
#     if empty_index is None:
#         empty_index = 0
#     else:
#         empty_index = len(df)
    
#     # Append the new value to the next available row in the specified column
#     df.loc[empty_index, column_name] = new_value
    
#     # Save the updated DataFrame back to the CSV
#     df.to_csv(file_path, index=False)
#     print(f"Added new value '{new_value}' to column '{column_name}' at row {empty_index}.")

# Example usage
columns = [
    'robot1_actuation_start', 'robot1_actuation_end','robot2_actuation_start','robot2_actuation_end', 'robot1_send_text', 'robot2_send_text.1', 
    'Server_recive_text1', 'Server_recive_text2', 'RRT_start', 'RRT_finish', 
    'server_2_robot1', 'server_2_robot2', 'robot1_recived_dir', 'robot2_recived_dir'
]

# # Create an empty DataFrame with these columns
df = pd.DataFrame(columns=columns)
df.to_csv('/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv', index=False)


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
    print(f"Added new value '{new_value}' to column '{column_name}' at row {empty_index}.")


file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'
column_name = 'Server_recive_text2'  # Column to add data to
new_value = '956'  # New data to add in the next line

append_to_column(file_path, column_name, new_value)