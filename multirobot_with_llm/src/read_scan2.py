#!/usr/bin/env python3
import os
import time
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
from std_msgs.msg import String
import datetime
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from custom_llama import infer
from actionlib_msgs.msg import GoalStatusArray
import json
from llama_cpp import Llama
from langchain.llms.base import LLM
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from typing import Optional, List, Mapping, Any
import pandas as pd

rbot1_ldr = None
loc = None
robot1_status = 10
file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'

prv_scan_time = None

class LlamaLLM(LLM):
    model_path: str
    llm: Llama

    @property
    def _llm_type(self) -> str:
        return "llama-cpp-python"

    def __init__(self, model_path: str, **kwargs: Any):
        model_path = model_path
        llm = Llama(model_path=model_path)
        super().__init__(model_path=model_path, llm=llm, **kwargs)

    def _call(self, prompt: str, stop: Optional[List[str]] = None) -> str:
        response = self.llm(prompt, stop=stop or [])
        return response["choices"][0]["text"]

    @property
    def _identifying_params(self) -> Mapping[str, Any]:
        return {"model_path": self.model_path}  

def callback1(message1: LaserScan):
    global rbot1_ldr,prv_scan_time
    lidar = np.array(message1.ranges)
    #rbot1_ldr = message1.ranges
    lidar[np.isinf(lidar)] = -1 
    rbot1_ldr = np.append(lidar, -1)
    prv_scan_time = time.time()
    # save_data(rbot1_ldr, 'data/lidar_data.npy')


def callback2(message1: Odometry):
    global loc
   # print(message1.pose.pose.position)
    # save_data(np.array(message1.pose.pose.position), 'data/imu_data.npy')
    
    loc = 'robot loc : (' + str(int(round(message1.pose.pose.position.x))) + ',' + str(int(round(message1.pose.pose.position.y)))   +')'
    #loc = 'robot loc : (' + str(round(message1.pose.pose.position.x,2)) + ',' + str(round(message1.pose.pose.position.y,2))   +')'
    
    
# def callback3(message1):
#     global robot1_status
#     print(len(message1.status_list))
#     if(len(message1.status_list)>0):
#         robot1_status = message1.status_list.status
#         print(robot1_status)
#     # print(message1.status_list)
    
def callback3(msg):
    global robot1_status
    if(msg.status_list !=[]):
        robot1_status = msg.status_list[0].status


    
def publish_possible_directions(can_move):
   
    directions = np.array(['east', 'northeast', 'north', 'northwest', 'west', 'southwest', 'south', 'southeast'])
    # can_move = np.array([0, 0, 1, 0, 1, 1, 0, 0])
    
    # Filter the possible directions where the robot can move (can_move == 0)
    possible_directions = directions[can_move == 1]
    
    # Convert the numpy array to a comma-separated string for publishing
    possible_directions_str = ','.join(possible_directions)
    
    return possible_directions_str

def is_valid_json(output):
    try:
        json_object = json.loads(output)
        return True, json_object  # Output is valid JSON
    except json.JSONDecodeError as e:
        return False, str(e)

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
    
    global rbot1_ldr, rbot2_ldr,loc
    global robot1_status,prv_scan_time

   
    topic1 = '/jb_1/scan';
    topic2 = '/jb_1/odom';
    topic4 = '/jb_1/move_base/status'
    
    rospy.init_node('R2_scan_data',anonymous=True);

    rospy.Subscriber(topic1,LaserScan, callback1);
    rospy.Subscriber(topic2,Odometry, callback2);
    rospy.Subscriber(topic4,GoalStatusArray, callback3);

    pub = rospy.Publisher('/obstacle_array', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/jb_1/text_des', String, queue_size=10)
    pub3 = rospy.Publisher('/rb2_move_dir', String, queue_size=10)
    pub4 = rospy.Publisher('/robot2_text', String, queue_size=10)

    cor = True
    flag_1 = True

    
    # model = None
    # n_gpu_layers = 20
    # n_batch = 4

    # model = Llama(model_path="/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/models/llm/llama-2-13b-chat.Q4_K_M.gguf",
    #                        n_gpu_layers=n_gpu_layers,  # Uncomment to use GPU acceleration
    #                        # seed=1337, # Uncomment to set a specific seed
    #                        # n_ctx=n_
    #                        )   #llama-2-13b-chat.Q4_K_M mistral-7b-instruct-v0.2.Q8_0
    
    # # system_prompt = "### Instruction:Your task is to provide a json given the robot's location(robot loc) and Lidar data from the query, extract and format the "\
    # #                 "information as JSON. The Lidar data indicates robot can moving possibilities in eight directions "\
    # #                 "respectively: east,northeast,north,northwest,west,southwest,south,southeast. A value of 0 means an obstacle is present, while 1 means the path is clear. "\
    # #                  "Output Format:{{'coordinates': ['<robot location>'],'possible_directions': ['<list of all directions (out of the 8) where the robot can move, based on '1' values in the Lidar data array>']  }}" \
    # #                  "Example:" \
    # #                     "Input: 'robot loc : (3,4) and Lidar data : (1,0,0,0,0,1,1,0)'  " \
    # #                     "Output: {{'coordinates': ['3,4'], 'possible_directions': ['east','southwest','south']}}, " \
    # #                     "Input: 'robot loc : (10,5) and Lidar data : (1,0,1,0,0,0,1,1)' " \
    # #                     "Output: {{'coordinates': ['10,5'], 'possible_directions': ['east','south','southeast']}}  ," \
    # #                     "Input: 'robot loc : (102,32) and Lidar data : (1,1,1,1,1,1,1,1)'  "  \
    # #                     "Output: {{'coordinates': ['102,32'], 'possible_directions': ['east','northeast','north','northwest','west','southwest','south','southeast']}}  "  \
    # #                     "Using the above information provide only output of the model in only json file format no need any other informations" \
    # #                 " ### Input: {inp} ### Response: "
    
    # system_prompt = "### Instruction: Your task is to analyze the robot's location and an 8-value Lidar data array. The Lidar data indicates whether the robot can move in eight directions, respectively: east (1st value), northeast (2nd), north (3rd), northwest (4th), west (5th), southwest (6th), south (7th), and southeast (8th). "\
    #             "A value of 0 means an obstacle is present, while a value of 1 means the path is clear. "\
    #             "Based on this, provide only a description of the robot’s current location and its ability to move in the clear directions(All directons).This is direction list [east (1st value), northeast (2nd), north (3rd), northwest (4th), west (5th), southwest (6th), south (7th), and southeast (8th)]. No JSON output is needed. "\
    #             "For example:" \
    #             "Input: 'robot loc: (0, 0) and Lidar data: (0,0,0,0,0,0,0,0)' " \
    #             "Output: 'My current location is (0,0). I can’t move in any direction.' " \
    #             "Input: 'robot loc: (0,0) and Lidar data: (0,0,0,0,0,0,0,1)' " \
    #             "Output: 'My current location is (0,0). I can move only in the southeast direction.' " \
    #             "Input: 'robot loc: (0,0) and Lidar data: (0,0,0,0,0,0,1,1)' " \
    #             "Output: 'My current location is (0,0). I can move in the south and southeast directions.' " \
    #             "Input: 'robot loc: (0,0) and Lidar data: (1,1,0,0,0,1,1,1)' " \
    #             "Output: 'My current location is (0,0). I can move in the east,northeast,southwest, south, and southeast directions.' "\
    #             "Please provide only the description, nothing else. "\
    #             "### Input: {inp} ### Response: "
    

    while not rospy.is_shutdown():
        
        
        if rbot1_ldr is not None and loc is not None:
            
            if(flag_1==True or robot1_status==3 ):
                # print("sdghis")
                ldata_45 = []
                ldata_all = []
            
                element1 = np.concatenate((rbot1_ldr[337:360], rbot1_ldr[:22]))
                element2 = rbot1_ldr[22:67] 
                element3 = rbot1_ldr[67:112]                
                element4 = rbot1_ldr[112:157] 
                element5 = rbot1_ldr[157:202] 
                element6 = rbot1_ldr[202:247] 
                element7 = rbot1_ldr[247:292] 
                element8 = rbot1_ldr[292:337] 
            
                ldata_all.append(element1)
                ldata_all.append(element2)
                ldata_all.append(element3)
                ldata_all.append(element4)
                ldata_all.append(element5)
                ldata_all.append(element6)
                ldata_all.append(element7)
                ldata_all.append(element8)
                
                ldata_all = np.asarray(ldata_all)
                obstcle = np.zeros(8)
                for i in range (len(ldata_all)):
                    count = 0
                    count2 = 0
                    dis_sum = 0
                    for j in range(len(ldata_all[i])):
                        if(ldata_all[i][j]==-1):
                            count = count + 1
                            
                        else:
                            count2 = count2 +1
                            dis_sum = dis_sum + ldata_all[i][j]
                    
                    if(count2>0):
                        dis_avg = dis_sum/count2
                    
                    if(count>34 or dis_avg >=3):
                        obstcle[i] = 1
                        
                    elif(dis_avg<3):
                        obstcle[i] = 0
                        
                array_msg = Float64MultiArray()
                array_msg.data = obstcle
                pub.publish(array_msg)
                
                lidar_data = 'Lidar data : (' + str(obstcle[0]) +','+ str(obstcle[1]) +','+ str(obstcle[2]) +','+ str(obstcle[3]) +','+ str(obstcle[4]) +','+ str(obstcle[5]) +','+ str(obstcle[6]) +','+ str(obstcle[7]) +')'
                # print(loc)
                # print(lidar_data)
                
                in_str = loc + " " + lidar_data
                dir = publish_possible_directions(obstcle)
                msg = String()
                msg.data = dir
                # print(dir)
                if((cor ==True and (robot1_status==3 or robot1_status==4) ) or flag_1):
                    #print("robot 1 direction send scan: " ,dir)
                    output = {
                    "coordinates": loc,
                    "possible_directions": lidar_data
                    }
                    
                    #print("robot 1 loc: ", loc)
                    #print("robot 2 publish textual description")
                    # print(lidar_data)


                    # user_query = "'''" + in_str + "'''"

                    # instruction = system_prompt.format(inp=in_str)

                    # outp = model(
                    #     instruction,  # Prompt
                    #     max_tokens=250,  # Generate up to 32 tokens, set to None to generate up to the end of the context window
                    #     stop=["### Input"],  # Stop generating just before the model would generate a new question
                    #     echo=True  # Echo the prompt back in the output
                    # )  # Generate a completion, can also call create_completion

                    # result = outp['choices'][0]['text'].split("### Response:")[-1]
                    # # print('result ' + str(result))
                    
                    # print(result)
                    # result = result.replace("\'", "\"")
                    # time.sleep(6)
                   

                    column_name = 'rbt2_fov'  
                    new_value = str(time.time()-prv_scan_time)
                    #append_to_column(file_path, column_name, new_value)

                    if(robot1_status==3 or robot1_status==4):
                        column_name = 'rbt2_actuation_finish'  
                        new_value = str(time.time())
                        #append_to_column(file_path, column_name, new_value)

                    time.sleep(0.5)

                    column_name = 'rbt2_2_server'  
                    new_value = str(time.time())
                    #append_to_column(file_path, column_name, new_value)
                    print(output)
                    for i in range(100):
                        # print(out)
                        pub2.publish(str(output))
                        pub3.publish(msg)
                        
                        
                    cor = False
                    flag_1 = False
                    
            elif(robot1_status!=3):
                cor = True
            
        
            

                    
                        



if __name__ == '__main__':
    
    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;