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


    llm = LlamaLLM(model_path="/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/models/llm/llama-2-7b-chat.Q4_K_M.gguf")
    
    prompt = PromptTemplate(
    input_variables=["query","instruction"],
    template='''\n\n### Instruction:"{instruction}"  
    ### Task:
    Given the robot's location and Lidar data from the query, extract and format the information as JSON. The Lidar data indicates robot can moving possibilities in eight directions respectively: east,northeast,north,northwest,west,southwest,south,southeast. A value of 0 means an obstacle is present, while 1 means the path is clear.
    ### Output Format:
    {{
        "coordinates": ["<robot location>"],
        "possible_directions": ["<list of directions with no obstacles. directions where the robot can move.only include directions corresponding to '1' values in the Lidar data>"]
    }}
    ### Example:
    Input: "robot loc : (3,4) and Lidar data : (1,0,0,0,0,1,1,0)"
    Output: {{"coordinates": ["3,4"], "possible_directions": ["east","southwest",south"]}}

    Input: "robot loc : (10,5) and Lidar data : (1,0,1,0,0,0,1,1)"
    Output: {{"coordinates": ["10,5"], "possible_directions": ["east","south","southeast"]}}

    Input: "robot loc : (102,32) and Lidar data : (1,1,1,1,1,1,1,1)"
    Output: {{"coordinates": ["102,32"], "possible_directions": ["east","northeast","north","northwest","west","southwest","south","southeast"]}}
   
    Using the above information provide only output of the model in only json file format no need any other informations
    \n\n### Response:\n
    '''
    )
    
    
    chain = LLMChain(llm=llm, prompt=prompt)

    user_query = "'''" + str_input + "'''"
    
    


    output = chain.run(user_query)
    
    print(output)

    
    end_time = time.time()  # Stop measuring time
    inference_time = end_time - start_time
    print("Inference Time:", inference_time)  # P
   
    output = chain.run(user_query)
    
    print(output)
 
    
    end_time = time.time()  # Stop measuring time
    inference_time = end_time - start_time
    print("Inference Time:", inference_time)
    
    return output
if __name__ == "__main__":

    #tst = 'Please go to the coffee room and see if the robot is there'
    tst = 'robot loc : (10,5) and Lidar data : (1,1,1,1,1,1,1,1)'
    #(1,1,1,1,0,0,0,0)'
    infer(tst)
