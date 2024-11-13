#!/usr/bin/env python3
import ast
import json
from llama_cpp import Llama
import time
# from openai import OpenAI
input_file = '/home/icon-group/Documents/dataset2.txt'
output_file = '/home/icon-group/Documents/inference_results2.txt'

    
def infer(user_prompt):
      #llama-2-13b-chat.Q4_K_M mistral-7b-instruct-v0.2.Q8_0  llama-2-70b.Q4_K_M
    
    # system_prompt = "### Instruction:Your task is to provide a json given the robot's location(robot loc) and Lidar data from the query, extract and format the "\
    #                 "information as JSON. The Lidar data indicates robot can moving possibilities in eight directions "\
    #                 "respectively: east,northeast,north,northwest,west,southwest,south,southeast. A value of 0 means an obstacle is present, while 1 means the path is clear. "\
    #                  "Output Format:{{'coordinates': ['<robot location>'],'possible_directions': ['<list of all directions (out of the 8) where the robot can move, based on '1' values in the Lidar data array>']  }}" \
    #                  "Example:" \
    #                     "Input: 'robot loc : (3,4) and Lidar data : (1,0,0,0,0,1,1,0)'  " \
    #                     "Output: {{'coordinates': ['3,4'], 'possible_directions': ['east','southwest','south']}}, " \
    #                     "Input: 'robot loc : (10,5) and Lidar data : (1,0,1,0,0,0,1,1)' " \
    #                     "Output: {{'coordinates': ['10,5'], 'possible_directions': ['east','south','southeast']}}  ," \
    #                     "Input: 'robot loc : (102,32) and Lidar data : (1,1,1,1,1,1,1,1)'  "  \
    #                     "Output: {{'coordinates': ['102,32'], 'possible_directions': ['east','northeast','north','northwest','west','southwest','south','southeast']}}  "  \
    #                     "Using the above information provide only output of the model in only json file format no need any other informations" \
    #                 " ### Input: {inp} ### Response: "
    
    # system_prompt = "### Instruction: Your task is to provide a valid JSON given the input query. The input query contains the robot's location and an 8-value Lidar data array. The Lidar data indicates whether the robot can move in eight directions, respectively: east (1st value), northeast (2nd), north (3rd), northwest (4th), west (5th), southwest (6th), south (7th), and southeast (8th). "\
    #             "A value of 0 means an obstacle is present, while a value of 1 means the path(direction) is clear. "\
    #             "Output a JSON in the following format: "\
    #             "{{'coordinates': ['<robot location>'], 'possible_directions': ['<list of directions where the robot can move based on 1 values in Lidar data (input query)>']}}" \
    #             "For example:" \
    #             "Input: 'robot loc: (13,41) and Lidar data: (1,0,0,0,0,0,0,0)' " \
    #             "Output: {{'coordinates': ['3,4'], 'possible_directions': ['east']}}. " \
    #             "Input: 'robot loc: (3,4) and Lidar data: (1,0,0,0,0,1,1,0)' " \
    #             "Output: {{'coordinates': ['3,4'], 'possible_directions': ['east','southwest','south']}}. " \
    #             "Input: 'robot loc: (102,32) and Lidar data: (1,1,1,1,1,1,1,1)' " \
    #             "Output: {{'coordinates': ['102,32'], 'possible_directions': ['east','northeast','north','northwest','west','southwest','south','southeast']}}. "\
    #             "Please provide the output in JSON format, and ensure the directions correspond correctly to the Lidar values. No other information is required. "\
    #             "### Input: {inp} ### Response: "

# "Input: 'robot loc: (10,5) and Lidar data: (1,0,1,0,0,0,1,1)' " \
#                 "Output: {{'coordinates': ['10,5'], 'possible_directions': ['east','north','south','southeast']}}. " \
    
    # system_prompt = """### Instruction:
    #                     You are given the robot's current location (robot loc) and its Lidar data. The Lidar data indicates the robot's possible movement in eight directions: east, northeast, north, northwest, west, southwest, south, and southeast. Each direction has a corresponding value: 0 means there is an obstacle, and 1 means the path is clear.
    #                     Your task is to extract the robot's location and possible movement directions, then format them as a JSON object.
    #                     ### JSON Output Format:
    #                     {{
    #                     "coordinates": ["<robot location>"],
    #                     "possible_directions": ["<list of all directions (out of the 8) where the robot can move, based on '1' values in the Lidar data array>"]
    #                     }}
    #                     ### Examples:
    #                     Input: 'robot loc : (3,4) and Lidar data : (1,0,0,0,0,1,1,0)'
    #                     Output: {{"coordinates": ["3,4"], "possible_directions": ["east","southwest","south"]}}
    #                     Input: 'robot loc : (10,5) and Lidar data : (1,0,1,0,0,0,1,1)'
    #                     Output: {{"coordinates": ["10,5"], "possible_directions": ["east","north","south","southeast"]}}
    #                     Input: 'robot loc : (102,32) and Lidar data : (1,1,1,1,1,1,1,1)'
    #                     Output: {{"coordinates": ["102,32"], "possible_directions": ["east","northeast","north","northwest","west","southwest","south","southeast"]}}
    #                     Using the above information, provide only the output in JSON format. Do not include any additional information.
    #                     ### Input: {inp} ### Response:
    #                     """



    # system_prompt = "### Instruction: Your task is to analyze the robot's location and an 8-value Lidar data array. The Lidar data indicates whether the robot can move in eight directions, respectively: east (1st value), northeast (2nd), north (3rd), northwest (4th), west (5th), southwest (6th), south (7th), and southeast (8th). "\
    #             "A value of 0 means an obstacle is present, while a value of 1 means the path is clear(direction). "\
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



    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:

        

        for line in infile:
            model = None
            n_gpu_layers = 20
            n_batch = 4

            model = Llama(model_path="/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/models/llm/llama-2-7b-chat.Q4_K_M.gguf",
                            n_gpu_layers=n_gpu_layers,  # Uncomment to use GPU acceleration
                            # seed=1337, # Uncomment to set a specific seed
                            # n_ctx=n_
                            ) 




            system_prompt = "### Instruction:Given the robot's location and Lidar data from the query, extract and format the information as JSON. The Lidar data indicates robot can moving possibilities in eight directions respectively: east,northeast,north,northwest,west,southwest,south,southeast."\
                    "Output Format:{{'coordinates': ['<robot location>'],'possible_directions': ['<extract directions using input query>']}}"\
                    "For example:" \
                    "Input: 'My current location is (3,4). I can’t move in any direction.' " \
                    "Output: '{{'coordinates': ['3,4'], 'possible_directions': []}}' " \
                    "Input: 'My current location is (0,0). I can move only in the southeast direction.' " \
                    "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['southeast']}}' " \
                    "Input: 'My current location is (0,0). I can move in the south and southeast directions.' " \
                    "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['south,southeast']}}' " \
                    "Input: 'My current location is (0,0). I can move in the east,northeast,southwest, south, and southeast directions.' " \
                    "Output: '{{'coordinates': ['0,0'], 'possible_directions': ['east,northeast,southwest, south, southeast']}}"\
                    "Using the above information provide only output of the model in only json file format no need any other informations"\
                    "### Input: {inp} ### Response: "

            start_time = time.time()  # Track inference start time
            instruction = system_prompt.format(inp=str(line))
            
            # Perform inference on the line
            output = model(
                    instruction,  # Prompt
                    max_tokens=500,  # Generate up to 32 tokens, set to None to generate up to the end of the context window
                    stop=["### Input"],  # Stop generating just before the model would generate a new question
                    echo=True  # Echo the prompt back in the output
                )  # Generate a completion, can also call create_completion

            
            # Calculate inference time
            infer_time = time.time() - start_time
            
            # Write result and time to output file
            outfile.write(str(infer_time) + "\n")


    

    
    result = output['choices'][0]['text'].split("### Response:")[-1]
    # print('result ' + str(result))
    
    # print(result)
    # result = result.replace("\'", "\"")
    try:
        json_object = json.loads(result)
        print("True")
        return True, json_object  # Output is valid JSON
    except json.JSONDecodeError as e:
        print("False")
        return False, str(e)
    
    # llm_json = json.loads(result)


    
    return output


if __name__ == "__main__":

    #tst = 'Please go to the coffee room and see if the robot is there'
    tst = "'robot loc' : (100,5) , 'Lidar data' : (0,1,0,1,1,1,1,1)"
    #(1,1,1,1,0,0,0,0)'
    infer(tst)
