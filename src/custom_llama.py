#!/usr/bin/env python3
import time
import json
from llama_cpp import Llama
from langchain.llms.base import LLM
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain
from typing import Optional, List, Mapping, Any


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
    
def infer(str_input):
    start_time = time.time()


    llm = LlamaLLM(model_path="/home/josh/catkin_ws/src/multi_robot/multibot_gai/src/model/llama-2-13b-chat.Q4_K_M.gguf")
    #llm = LlamaLLM(model_path="/home/icon-group/chat_bot/chat_project/chat_app/models/mistral-7b-instruct-v0.2.Q8_0.gguf")



    # prompt = PromptTemplate(
    # input_variables=["query"],
    # template='''\n\n### Instruction:\nGiven the instruction "{instruction}", extract the following information: 
    # So example input query and model output are below. Please refer to these examples and provide only model output in the following format
    # input: "robot loc : (3,4) and  Lidar data : (1,0,0,0,0,1,1,0)", Output: [" coordinates (3, 4) possible directions:East,Southwest,South"]
    # input: "robot loc : (10,5) and  Lidar data : (1,0,1,0,0,0,1,1)", Output: [" coordinates (10, 5) possible directions:East,North,South,Southeast"]
    # OUt putformat is include the location and directions 
    # This is the input and output format. The LIDAR data represents the possible directions for the robot to move. A value of 0 indicates that movement is not possible, while a value of 1 indicates that movement is possible. The LIDAR data covers 8 directions: east, northeast, north, and so on, covering all 8 compass directions.
    # only output and no need more response
    #  \n\n### Response:\n'''
    # )

    # prompt = PromptTemplate(
    # input_variables=["query"],
    # template='''\n\n###Given the instruction "{instruction}", extract the following information from the input query and provide it in JSON format.
    # Lidar data represents eight directions respectively north,northeast,east,southeast,south,southwest,west,northwest. A value of 1 indicates that there is an obstacle in that direction, while a value of 0 indicates no obstacle, allowing the robot to move in that direction.
    # output format:
    # {{
    # "coordinates": ["<robot location>"],
    # "possible_directions": ["<directions where the robot can move. Only include directions corresponding to '0' values in the Lidar data>>"]
    # }}
    
    # So example input query and model output are below. Please refer to these examples and provide only model output in the following format.
    # input: "robot loc : (3,4) and  Lidar data : (1,0,0,0,0,1,1,0)", Output: {{"coordinates": ["3,4"], "Posible_Directions": ["northeast","east","southeast",south"]}}.
    # input: "robot loc : (10,5) and  Lidar data : (1,0,1,0,0,0,1,1)", Output: {{"coordinates": ["10,5"], "Posible_Directions": ["northeast","southeast","south","southwest"]}}.
    # input: "robot loc : (102,32) and  Lidar data : (1,1,1,1,1,1,1,1)", Output: {{"coordinates": ["102,32"], "Posible_Directions": ["None"]}}.
    # Using the above information provide only output of the model in json file format
    #  \n\n### Response:\n'''
    # )
    
    prompt = PromptTemplate(
    input_variables=["query","instruction"],
    template='''\n\n### Instruction:"{instruction}"  
    ### Task:
    Given the robot's location and Lidar data from the query, extract and format the information as JSON. The Lidar data indicates obstacles in eight directions respectively: north,northeast,east,southeast,south,southwest,west,northwest. A value of 1 means an obstacle is present, while 0 means the path is clear.
    ### Output Format:
    {{
        "coordinates": ["<robot location>"],
        "possible_directions": ["<list of directions with no obstacles. directions where the robot can move.only include directions corresponding to '0' values in the Lidar data>"]
    }}
    ### Example:
    Input: "robot loc : (3,4) and Lidar data : (1,0,0,0,0,1,1,0)"
    Output: {{"coordinates": ["3,4"], "possible_directions": ["northeast","east","southeast",south"]}}

    Input: "robot loc : (10,5) and Lidar data : (1,0,1,0,0,0,1,1)"
    Output: {{"coordinates": ["10,5"], "possible_directions": ["northeast","southeast","south","southwest"]}}

    Input: "robot loc : (102,32) and Lidar data : (1,1,1,1,1,1,1,1)"
    Output: {{"coordinates": ["102,32"], "possible_directions": ["north","northeast","east","southeast","south","southwest","west","northwest"]}}
    Using the above information provide only output of the model in only json file format no need any other informations
    \n\n### Response:\n
    '''
    )
    
    
    chain = LLMChain(llm=llm, prompt=prompt)
    #str_input = "Could you please go to the robotics lab and check if the robot is there?"
    # Run the chain only specifying the input variable.
    user_query = "'''" + str_input + "'''"
    
    


    output = chain.run(user_query)
    
    print(output)
    # output_data = json.loads(output)
    # print(output_data)
    
    end_time = time.time()  # Stop measuring time
    inference_time = end_time - start_time
    print("Inference Time:", inference_time)  # P
   
    
    return output
if __name__ == "__main__":

    #tst = 'Please go to the coffee room and see if the robot is there'
    tst = 'robot loc : (10,5) and Lidar data : (0,0,0,0,0,0,1,1)'
    #(1,1,1,1,0,0,0,0)'
    infer(tst)
