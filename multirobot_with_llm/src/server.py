#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
from geometry_msgs.msg import Pose
import tf2_ros
import geometry_msgs.msg
import math
import time
from llama_cpp import Llama
from langchain.llms.base import LLM
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from typing import Optional, List, Mapping, Any
import pandas as pd


text_des_1 = None
text_des_2 = None

file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'



def callback1(data):
    global text_des_1
    text_des_1 = data.data
    
    #print("recived location1")

    
def callback2(data):
    global text_des_2
    text_des_2 = data.data
    
    #print("recived location2")
    
def convert_json(str_text):  
     
    str_text = str_text.replace("'", '"')

    data_dict = json.loads(str_text)


    coords_str = data_dict["coordinates"].split(":")[1].strip().strip("()")
    coords = [float(x) for x in coords_str.split(",")]

    # Extract Lidar data
    lidar_str = data_dict["possible_directions"].split(":")[1].strip().strip("()")
    lidar_data = [float(x) for x in lidar_str.split(",")]

    json_output = {
        "coordinates": coords,
        "possible_directions": lidar_data
    }

    json_result = json.dumps(json_output, indent=4)
    
    return json_result


def mark_explored(obstacle, square_size, grid_data, robot_loc, width, height):
    half_size = square_size // 2
    
    directions = [
        (1, 1),   
        (-1, -1),
        (-1,1),
        (1,-1)   
        
    ]
    top_left_x = robot_loc[0] + directions[2][0] *half_size
    top_left_y = robot_loc[1] + directions[2][1] *half_size
    
    top_right_x = robot_loc[0] + directions[0][0] *half_size
    top_right_y = robot_loc[1] + directions[0][1] *half_size
    
    bottom_right_x = robot_loc[0] + directions[1][0] *half_size
    bottom_right_y = robot_loc[1] + directions[1][1] *half_size
    
    bottom_left_x = robot_loc[0] + directions[3][0] *half_size
    bottom_left_y = robot_loc[1] + directions[3][1] *half_size
    
    for i in range(bottom_left_y,top_left_y):
        for j in range(top_left_x,top_right_x):
            index = j +  i*width
            # if(i==bottom_left_y or i== bottom_left_y-1 or j==top_left_x or j==top_left_x-1):
            #      if(grid_data[index]==-1):
            #         grid_data[index] = 0 
            
            if(grid_data[index]==-1):

                grid_data[index] = 0 
            
    return grid_data
          

def mark_obstacles(obstacle, square_size, grid_data, robot_loc, width, height):

    half_size = square_size // 2
    ratio = square_size // 4
    directions = [
        (1, 0),   # East
        (1, 1),   # North-East
        (0, 1),   # North
        (-1, 1),  # North-West
        (-1, 0),  # West
        (-1, -1), # South-West
        (0, -1),  # South
        (1, -1)   # South-East
    ]
    
    for k, is_free in enumerate(obstacle):
           
        if(k==0 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_y-ratio,obstacle_y+ratio):
                    index = obstacle_x +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100 
                    
        if(k==1 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-ratio,obstacle_y):
                    index = obstacle_x +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
            for i in range(obstacle_x-ratio,obstacle_x+1):
                    index = i +  obstacle_y*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100 
                    
        if(k==2 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-ratio,obstacle_x+ratio):
                    index = i +  obstacle_y*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
                    
        if(k==3 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-ratio,obstacle_y):
                    index = obstacle_x-1 +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100 
            for i in range(obstacle_x-1,obstacle_x+ratio):
                    index = i +  obstacle_y*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
                    
        if(k==4 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_y-ratio,obstacle_y+ratio):
                    index = obstacle_x-1 +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
                    
        if(k==5 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+ratio):
                    index = obstacle_x-1 +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
            for i in range(obstacle_x-1,obstacle_x+ratio):
                    index = i +  (obstacle_y-1)*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100 
        if(k==6 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-ratio,obstacle_x+ratio):
                    index = i +  (obstacle_y-1)*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
                    
        if(k==7 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+ratio):
                    index = obstacle_x +  i*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
            for i in range(obstacle_x-ratio,obstacle_x+1):
                    index = i +  (obstacle_y-1)*width
                    if(grid_data[index]==-1):
                        grid_data[index] = 100
            
    
    
    return grid_data


def compute_costmap(ogm, inflation_radius=1, cost_scaling_factor=3):
    # Convert the OccupancyGrid data to a NumPy array
    width = ogm.info.width
    height = ogm.info.height
    ogm_data = np.array(ogm.data).reshape((height, width))

    # Initialize the costmap
    costmap = np.copy(ogm_data)

    # Apply inflation around each obstacle
    for r in range(height):
        for c in range(width):
            if ogm_data[r, c] == 100:  # Obstacle detected
                for i in range(-inflation_radius, inflation_radius + 1):
                    for j in range(-inflation_radius, inflation_radius + 1):
                        # Check that the neighboring cell is within grid bounds
                        if 0 <= r + i < height and 0 <= c + j < width:
                            distance = np.sqrt(i**2 + j**2)
                            if distance <= inflation_radius:
                                # Apply a cost based on the distance to the obstacle
                                cost = int(cost_scaling_factor * (1 - (distance / inflation_radius)))
                                costmap[r + i, c + j] = min(255, max(costmap[r + i, c + j], cost))

    return costmap


def calculate_angle_and_direction(prv_abstrct_loc, rbt_loc,prv_rbt_loc,square_size): #prv_loc1, robot_loc2_act,prv_robot_loc1,square_size
    print(rbt_loc)
    print(prv_rbt_loc)
    x1, y1 = prv_rbt_loc
    x2, y2 = rbt_loc
    new_loc = [0,0]
    # Calculate differences in coordinates
    dx = x2 - x1
    dy = y2 - y1

    # print(dx)
    # print(dy)
    
    DIRECTIONS = {
            0: (1, 0),    # Right
            45: (1, 1),   # Diagonal up-right
            90: (0, 1),   # Up
            135: (-1, 1), # Diagonal up-left
            180: (-1, 0), # Left
            225: (-1, -1),# Diagonal down-left
            270: (0, -1), # Down
            315: (1, -1)  # Diagonal down-right
        }

    if(abs(dx)>1 or abs(dy)>1):
    # Calculate angle in radians using atan2
        angle_rad = math.atan2(dy, dx)
        angle_deg = math.degrees(angle_rad)
        quantized_angle = round(angle_deg / 45) * 45
        
        quantized_angle = quantized_angle % 360
                
        dx1, dy1 = DIRECTIONS[quantized_angle]
        
        # quantized_angle_rad = math.radians(quantized_angle)

        # # Convert radians to degrees for easier interpretation
        # # angle_deg = math.degrees(angle_rad)
        # radius = square_size // 2
        # print(quantized_angle_rad)
        # dx1 = radius * math.cos(quantized_angle_rad)
        # dy1 = radius * math.sin(quantized_angle_rad)

        new_loc[0] = prv_abstrct_loc[0]+(dx1*square_size // 2)
        new_loc[1] = prv_abstrct_loc[1]+(dy1*square_size // 2)
    else:
        new_loc[0] = prv_abstrct_loc[0]
        new_loc[1] = prv_abstrct_loc[1]


    return new_loc


def save_data(file_path,data):

    with open(file_path, 'r') as file:
        content = file.readlines()

    # for line in content:
    #     print(line.strip())

    data = data+"\n"
    with open(file_path, 'a') as file:
        file.write(data)


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


def main():
    global text_des_1,text_des_2

    # model = None
    # n_gpu_layers = 20
    # n_batch = 4

    # model = Llama(model_path="/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/models/llm/llama-2-13b-chat.Q4_K_M.gguf",
    #                        n_gpu_layers=n_gpu_layers,  # Uncomment to use GPU acceleration
    #                        # seed=1337, # Uncomment to set a specific seed
    #                        # n_ctx=n_
    #                        )   #llama-2-13b-chat.Q4_K_M mistral-7b-instruct-v0.2.Q8_0
    

    
    
    # system_prompt = "### Instruction:Given the robot's location and Lidar data from the query, extract and format the information as JSON. The Lidar data indicates robot can moving possibilities in eight directions respectively: east,northeast,north,northwest,west,southwest,south,southeast."\
    #                 "Output Format:{{'coordinates': ['<robot location>'],'possible_directions': ['<extract directions using input query>']}}"\
    #                 "For example:" \
    #                 "Input: 'My current location is (3,4). I can’t move in any direction.' " \
    #                 "Output: '{{'coordinates': ['3,4'], 'possible_directions': []}}' " \
    #                 "Input: 'My current location is (0,0). I can move only in the southeast direction.' " \
    #                 "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['southeast']}}' " \
    #                 "Input: 'My current location is (0,0). I can move in the south and southeast directions.' " \
    #                 "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['south,southeast']}}' " \
    #                 "Input: 'My current location is (0,0). I can move in the east,northeast,southwest, south, and southeast directions.' " \
    #                 "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['east,northeast,southwest, south, southeast']}}"\
    #                 "Using the above information provide only output of the model in only json file format no need any other informations"\
    #                 "### Input: {inp} ### Response: "
    

    
    rospy.init_node('server', anonymous=True)
    rospy.Subscriber('/jb_0/text_des', String, callback1)
    rospy.Subscriber('/jb_1/text_des', String, callback2)
    
    pub1 = rospy.Publisher('/robot_1/map', OccupancyGrid, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/map', OccupancyGrid, queue_size=10)
    pub3 = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)
    
    costmap_pub1 = rospy.Publisher('/robot_1/costmap', OccupancyGrid, queue_size=10)
    costmap_pub2 = rospy.Publisher('/robot_2/costmap', OccupancyGrid, queue_size=10)

    grid1 = OccupancyGrid()
    grid1.header = Header()
    grid1.header.frame_id = "robot_1/map"
    grid1.info.resolution = 0.1  
    grid1.info.width = 1000
    grid1.info.height = 1000
    grid1.info.origin.position.x = 0
    grid1.info.origin.position.y = 0 
    grid1.info.origin.position.z = 0   
    grid1.data = [-1] * (grid1.info.width * grid1.info.height)
    
    
    grid2 = OccupancyGrid()
    grid2.header = Header()
    grid2.header.frame_id = "robot_2/map"
    grid2.info.resolution = 0.1  
    grid2.info.width = 1000
    grid2.info.height = 1000
    grid2.info.origin.position.x = 0 
    grid2.info.origin.position.y = 0  
    grid2.info.origin.position.z = 0   
    grid2.data = [-1] * (grid2.info.width * grid2.info.height)
    
    grid3 = OccupancyGrid()
    grid3.header = Header()
    grid3.header.frame_id = "global"
    grid3.info.resolution = 0.1  
    grid3.info.width = 1000
    grid3.info.height = 1000
    grid3.info.origin.position.x = 0  
    grid3.info.origin.position.y = 0  
    grid3.info.origin.position.z = 0   
    grid3.data = [-1] * (grid3.info.width * grid3.info.height)
    
    
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    static_transform_stamped.header.frame_id = "robot_1/map"  # Parent frame
    static_transform_stamped.child_frame_id = "robot_1/base_link"   #
    
    broadcaster2 = tf2_ros.StaticTransformBroadcaster()
    static_transform_stamped2 = geometry_msgs.msg.TransformStamped()
    static_transform_stamped2.header.frame_id = "robot_2/map"  # Parent frame
    static_transform_stamped2.child_frame_id = "robot_2/base_link"   #
  
    
    
    inflation_radius = 1  # Cells around obstacles to inflate
    cost_scaling_factor = 50  # Controls how steep the cost gradient is near obstacles
    
    
    square_size = 32
    #center2 = (int(grid1.info.width/2),int(grid3.info.height/2)) #+square_size//2
    #center = (516,484)
    #center = (int(grid1.info.width/2)-square_size,int(grid3.info.height/2))
    center = (500,516)
    center2 = (500,500)

    costmap_msg2 = OccupancyGrid()
    costmap_msg1 = OccupancyGrid()

    location1 = (0,0)
    location2 = (0,0)

    prv_loc1= None
    prv_loc2= None


    robot_loc1 = [0,0]
    robot_loc2 = [0,0]

    prv_robot_loc1 = [0,0]
    prv_robot_loc2 = [0,0]


    while not rospy.is_shutdown():
        
        
        if(text_des_1 is not None):

            strt_1 = time.time()

            column_name = 'server_rcv_1'  
            new_value = str(time.time())
            #umn(file_path, column_name, new_value)


            json_out1 = convert_json(text_des_1)
            json_out1 = json.loads(json_out1)
            robot_loc1_act = json_out1["coordinates"]
            obstacle1 = json_out1["possible_directions"]
            
            robot_loc1_act = [int(val) for val in robot_loc1_act]
            obstacle1 = [int(val) for val in obstacle1]
            obstacle1 = np.asarray(obstacle1)
            start_time = time.time()
            # tst = "'My current location is (0,0). I can move in the south and southeast directions."

            # instruction = system_prompt.format(inp=tst)

            # outp = model(
            #     instruction,  # Prompt
            #     max_tokens=500,  # Generate up to 32 tokens, set to None to generate up to the end of the context window
            #     stop=["### Input"],  # Stop generating just before the model would generate a new question
            #     echo=True  # Echo the prompt back in the output
            # )  # Generate a completion, can also call create_completion

            # result = outp['choices'][0]['text'].split("### Response:")[-1]


            if(prv_loc1 is not None):
                new_loc = calculate_angle_and_direction(prv_loc1, robot_loc1_act,prv_robot_loc1,square_size)
                robot_loc1[0] = int(new_loc[0])
                robot_loc1[1] = int(new_loc[1])

            else:
                
                robot_loc1[0] =center[0]
                robot_loc1[1] = center[1]


            location1 = robot_loc1  

            print("Robot 1location: ",robot_loc1)
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            finnish_1 = time.time()

            column_name = 'global_map'  
            new_value = str(finnish_1-strt_1)
            #append_to_column(file_path, column_name, new_value)


            prv_loc1 = robot_loc1
            # prv_loc1[1]  = robot_loc1[1]

            prv_robot_loc1[0]  = robot_loc1_act[0]
            prv_robot_loc1[1]  = robot_loc1_act[1]


            text_des_1 = None
        
        
        if(text_des_2 is not None):
            strt_2 = time.time()

            column_name = 'server_rcv_2'  
            new_value = str(time.time())
            #append_to_column(file_path, column_name, new_value)


            json_out2 = convert_json(text_des_2)
            json_out2 = json.loads(json_out2)
            robot_loc2_act = json_out2["coordinates"]
            obstacle2 = json_out2["possible_directions"] 
            
            robot_loc2_act = [int(val) for val in robot_loc2_act]
            obstacle2 = [int(val) for val in obstacle2]
            obstacle2 = np.asarray(obstacle2)
            print(obstacle2)
            start_time = time.time()
            # tst = "'My current location is (0,0). I can move in the south and southeast directions."
            # instruction = system_prompt.format(inp=tst)

            # outp = model(
            #     instruction,  # Prompt
            #     max_tokens=500,  # Generate up to 32 tokens, set to None to generate up to the end of the context window
            #     stop=["### Input"],  # Stop generating just before the model would generate a new question
            #     echo=True  # Echo the prompt back in the output
            # )  # Generate a completion, can also call create_completion

            # result = outp['choices'][0]['text'].split("### Response:")[-1]
            # end_time = time.time()

            # elapsed_time = end_time - start_time

            # file1 = "/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/server2_llm_out.txt"

            # data_1 = str(elapsed_time)

            # save_data(file1,data_1)

            if(prv_loc2 is not None):
                new_loc = calculate_angle_and_direction(prv_loc2, robot_loc2_act,prv_robot_loc2,square_size)
                robot_loc2[0] = int(new_loc[0])
                robot_loc2[1] = int(new_loc[1])

            else:
                robot_loc2[0] = center2[0]
                robot_loc2[1] = center2[1]

            
            print("Robot 2 location: ",robot_loc2)
            
            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
            finnish_2 = time.time()
            
            column_name = 'global_map'  
            new_value = str(finnish_2-strt_2)
            #append_to_column(file_path, column_name, new_value)
            
            prv_loc2  = robot_loc2
            # prv_loc2[1]  = robot_loc2[1]

            prv_robot_loc2[0]  = robot_loc2_act[0]
            prv_robot_loc2[1]  = robot_loc2_act[1]

            text_des_2 = None
            
        
        static_transform_stamped.transform.translation.x = location1[0] *grid3.info.resolution 
        static_transform_stamped.transform.translation.y = location1[1] *grid3.info.resolution 
        static_transform_stamped.transform.translation.z = 0.0

        # Set rotation (as quaternion: x, y, z, w)
        static_transform_stamped.transform.rotation.x = 0.0
        static_transform_stamped.transform.rotation.y = 0.0
        static_transform_stamped.transform.rotation.z = 0.0 # 90-degree around z-axis
        static_transform_stamped.transform.rotation.w = 1
        
        broadcaster.sendTransform(static_transform_stamped)
            
            
            
        static_transform_stamped2.transform.translation.x = location2[0] *grid3.info.resolution 
        static_transform_stamped2.transform.translation.y = location2[1] *grid3.info.resolution 
        static_transform_stamped2.transform.translation.z = 0.0

        # Set rotation (as quaternion: x, y, z, w)
        static_transform_stamped2.transform.rotation.x = 0.0
        static_transform_stamped2.transform.rotation.y = 0.0
        static_transform_stamped2.transform.rotation.z = 0.0 # 90-degree around z-axis
        static_transform_stamped2.transform.rotation.w = 1
        
        # for i in range(5):
        
        broadcaster2.sendTransform(static_transform_stamped2)
            
            
        costmap1 = compute_costmap(grid1, inflation_radius, cost_scaling_factor)
            
        costmap_msg1 = OccupancyGrid()

        costmap_msg1.header = Header(stamp=rospy.Time.now(), frame_id="robot_1/map")
        costmap_msg1.info.width = grid1.info.width
        costmap_msg1.info.height = grid1.info.height
        costmap_msg1.info.resolution = 0.1  # Set appropriate resolution (modify as needed)
        costmap_msg1.info.origin = Pose()  # Set origin (modify according to your map setup)

        costmap_msg1.data = costmap1.flatten().tolist()
        
        costmap_msg1.header.stamp = rospy.Time.now()  # Update timestamp
        for i in range(5):
            costmap_pub1.publish(costmap_msg1)
            
        # for i in range(5):
        
        
        costmap2 = compute_costmap(grid2, inflation_radius, cost_scaling_factor)
            
        costmap_msg2 = OccupancyGrid()

        costmap_msg2.header = Header(stamp=rospy.Time.now(), frame_id="robot_2/map")
        costmap_msg2.info.width = grid2.info.width
        costmap_msg2.info.height = grid2.info.height
        costmap_msg2.info.resolution = 0.1  # Set appropriate resolution (modify as needed)
        costmap_msg2.info.origin = Pose()  # Set origin (modify according to your map setup)

        costmap_msg2.data = costmap2.flatten().tolist()
        
        costmap_msg2.header.stamp = rospy.Time.now()  # Update timestamp
        for i in range(5):
            costmap_pub2.publish(costmap_msg2)
                
                
        grid1.header.stamp = rospy.Time.now()
        pub1.publish(grid1)
        
        grid2.header.stamp = rospy.Time.now()
        pub2.publish(grid2)
        
        grid3.header.stamp = rospy.Time.now()
        pub3.publish(grid3)
            

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
