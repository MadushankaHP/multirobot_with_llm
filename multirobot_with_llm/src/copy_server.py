#!/usr/bin/env python

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
from std_msgs.msg import Int32


text_des_1 = None
text_des_2 = None
value_1  = 1


def callback1(data):
    global text_des_1
    text_des_1 = data.data
    print("recived location1")

    
def callback2(data):
    global text_des_2
    text_des_2 = data.data
    print("recived location2")
    
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
                    grid_data[index] = 100 
                    
        if(k==1 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-ratio,obstacle_y):
                    index = obstacle_x +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-ratio,obstacle_x+1):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==2 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-ratio,obstacle_x+ratio):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==3 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-ratio,obstacle_y):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-1,obstacle_x+ratio):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==4 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_y-ratio,obstacle_y+ratio):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
                    
        if(k==5 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+ratio):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-1,obstacle_x+ratio):
                    index = i +  (obstacle_y-1)*width
                    grid_data[index] = 100 
        if(k==6 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-ratio,obstacle_x+ratio):
                    index = i +  (obstacle_y-1)*width
                    grid_data[index] = 100 
                    
        if(k==7 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+ratio):
                    index = obstacle_x +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-ratio,obstacle_x+1):
                    index = i +  (obstacle_y-1)*width
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
    x1, y1 = prv_rbt_loc
    x2, y2 = rbt_loc
    new_loc = [0,0]
    # Calculate differences in coordinates
    dx = x2 - x1
    dy = y2 - y1

    if(dx>0 or dy>0):
    # Calculate angle in radians using atan2
        angle_rad = math.atan2(dy, dx)

        # Convert radians to degrees for easier interpretation
        # angle_deg = math.degrees(angle_rad)
        radius = square_size // 2

        dx1 = radius * math.cos(angle_rad)
        dy1 = radius * math.sin(angle_rad)

        new_loc[0] = prv_abstrct_loc[0]+dx1
        new_loc[1] = prv_abstrct_loc[1]+dy1
    
    else:
        new_loc[0] = prv_abstrct_loc[0]
        new_loc[1] = prv_abstrct_loc[1]


    return new_loc
    
    
def callback(data):
    global value_1 
    value_1 = data.data

def main():
    global text_des_1,text_des_2
    global value_1 

    
    rospy.init_node('server', anonymous=True)
    rospy.Subscriber('/jb_0/text_des', String, callback1)
    rospy.Subscriber('/jb_1/text_des', String, callback2)
    rospy.Subscriber('/int_val', Int32, callback)
    
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
    center = (int(grid1.info.width/2),int(grid3.info.height/2))
    center2 = (int(grid1.info.width/2)-square_size//2,int(grid3.info.height/2)+square_size//2)
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
        
        while(value_1 ==1):
            

            robot_loc1 = [500,500]

            location1 = robot_loc1  
            obstacle1 = (0,1,1,1,0,0,0,0)
            obstacle1 = np.asarray(obstacle1)

            
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            
            

                
            robot_loc2 = [468,516]
            location1 = robot_loc1  
            obstacle2 = (1,1,0,1,1,1,1,1)
            obstacle2 = np.asarray(obstacle2)

            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
                
            
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
            costmap_msg1.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            costmap_msg2.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            
        
        
        ################################
        
        while(value_1 == 2 ):
        
            robot_loc1 = [500,516]

            location1 = robot_loc1  
            obstacle1 = (0,0,0,0,0,1,1,1)
            obstacle1 = np.asarray(obstacle1)

            
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            
            

                
            robot_loc2 = [468,500]
            location1 = robot_loc1  
            obstacle2 = (1,1,1,1,1,0,0,1)
            obstacle2 = np.asarray(obstacle2)

            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
                
            
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
            costmap_msg1.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            costmap_msg2.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
        
         ################################
        
        while(value_1 == 3 ):
        
            robot_loc1 = [500,516]

            location1 = robot_loc1  
            obstacle1 = (0,0,0,0,0,1,1,1)
            obstacle1 = np.asarray(obstacle1)

            
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            
            

                
            robot_loc2 = [484,500]
            location1 = robot_loc1  
            obstacle2 = (0,0,1,1,1,1,0,0)
            obstacle2 = np.asarray(obstacle2)

            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
                
            
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
            costmap_msg1.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            costmap_msg2.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            
            
         ################################
        
        while(value_1 == 4 ):
        
            robot_loc1 = [500,516]

            location1 = robot_loc1  
            obstacle1 = (0,0,0,0,0,1,1,1)
            obstacle1 = np.asarray(obstacle1)

            
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            
            

                
            robot_loc2 = [468,516]
            location1 = robot_loc1  
            obstacle2 = (1,1,0,1,1,1,1,1)
            obstacle2 = np.asarray(obstacle2)

            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
                
            
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
            costmap_msg1.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            costmap_msg2.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
        
        while(value_1 == 5 ):
        
            robot_loc1 = [500,516]

            location1 = robot_loc1  
            obstacle1 = (1,0,0,0,1,1,1,1)
            obstacle1 = np.asarray(obstacle1)

            
            
            

            grid1.data = mark_explored(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)
            grid1.data = mark_obstacles(obstacle1, square_size, grid1.data, robot_loc1, grid1.info.width, grid1.info.height)

            grid3.data = mark_explored(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle1, square_size, grid3.data, robot_loc1, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)

            
            

                
            robot_loc2 = [452,516]
            location1 = robot_loc1  
            obstacle2 = (1,0,0,0,1,1,1,1)
            obstacle2 = np.asarray(obstacle2)

            location2 = robot_loc2  
            
            
            
            grid2.data = mark_explored(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            grid2.data = mark_obstacles(obstacle2, square_size, grid2.data, robot_loc2, grid2.info.width, grid2.info.height)
            
            grid3.data = mark_explored(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            grid3.data = mark_obstacles(obstacle2, square_size, grid3.data, robot_loc2, grid3.info.width, grid3.info.height)
            
            grid3.header.stamp = rospy.Time.now()
            pub3.publish(grid3)
                
            
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
            costmap_msg1.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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
            costmap_msg2.info.resolution = 0.05  # Set appropriate resolution (modify as needed)
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