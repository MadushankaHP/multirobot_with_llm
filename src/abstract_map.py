#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np



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
            grid_data[index] = 0 
            
    return grid_data
    
    


def mark_obstacles(obstacle, square_size, grid_data, robot_loc, width, height):
    """
    Marks obstacles in the grid data based on the obstacle array.
    Each obstacle is drawn as a black line (100) on a section of the perimeter
    around the robot, divided into 8 equal segments.
    """
    half_size = square_size // 2
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
            for i in range(obstacle_y-2,obstacle_y+2):
                    index = obstacle_x +  i*width
                    grid_data[index] = 100 
                    
        if(k==1 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-2,obstacle_y):
                    index = obstacle_x +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-2,obstacle_x+1):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==2 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-2,obstacle_x+2):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==3 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y-2,obstacle_y):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-1,obstacle_x+2):
                    index = i +  obstacle_y*width
                    grid_data[index] = 100 
                    
        if(k==4 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_y-2,obstacle_y+2):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
                    
        if(k==5 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+2):
                    index = obstacle_x-1 +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-1,obstacle_x+2):
                    index = i +  (obstacle_y-1)*width
                    grid_data[index] = 100 
        if(k==6 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            for i in range(obstacle_x-2,obstacle_x+2):
                    index = i +  (obstacle_y-1)*width
                    grid_data[index] = 100 
                    
        if(k==7 and obstacle[k]==0):
            obstacle_x = robot_loc[0] + directions[k][0] *half_size
            obstacle_y = robot_loc[1] + directions[k][1] *half_size
            
            for i in range(obstacle_y,obstacle_y+2):
                    index = obstacle_x +  i*width
                    grid_data[index] = 100 
            for i in range(obstacle_x-2,obstacle_x+1):
                    index = i +  (obstacle_y-1)*width
                    grid_data[index] = 100 
            
    
    
    return grid_data


def mark_explored_area(grid_data, robot_loc, square_size, width, height):
    """
    Marks a 4x4 explored area centered around the robot as free space (value 0).
    """
    half_size = square_size // 2
    for i in range(max(0, robot_loc[0] - half_size), min(width, robot_loc[0] + half_size)):
        for j in range(max(0, robot_loc[1] - half_size), min(height, robot_loc[1] + half_size)):
            index = i + j * width
            grid_data[index] = 0  # Mark as free (explored)
    
    return grid_data


def draw_ogm():
    rospy.init_node('occupancy_grid_publisher', anonymous=True)

    pub1 = rospy.Publisher('/robot_1/map', OccupancyGrid, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/map', OccupancyGrid, queue_size=10)
    pub3 = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)

    grid1 = OccupancyGrid()
    grid1.header = Header()
    grid1.header.frame_id = "jb_0/map"
    grid1.info.resolution = 0.1  
    grid1.info.width = 200
    grid1.info.height = 200
    grid1.info.origin.position.x = -5  
    grid1.info.origin.position.y = -5  
    grid1.info.origin.position.z = 0   
    grid1.data = [-1] * (grid1.info.width * grid1.info.height)
    
    
    grid2 = OccupancyGrid()
    grid2.header = Header()
    grid2.header.frame_id = "jb_1/map"
    grid2.info.resolution = 0.1  
    grid2.info.width = 1000
    grid2.info.height = 1000
    grid2.info.origin.position.x = -5  
    grid2.info.origin.position.y = -5  
    grid2.info.origin.position.z = 0   
    grid2.data = [-1] * (grid2.info.width * grid2.info.height)
    
    grid3 = OccupancyGrid()
    grid3.header = Header()
    grid3.header.frame_id = "world"
    grid3.info.resolution = 0.1  
    grid3.info.width = 1000
    grid3.info.height = 1000
    grid3.info.origin.position.x = -5  
    grid3.info.origin.position.y = -5  
    grid3.info.origin.position.z = 0   
    grid3.data = [-1] * (grid3.info.width * grid3.info.height)
    
    
    
    
    
    

    square_size = 8  # Defines the size of the explored area
    obstacle = np.array([1, 0, 1, 0, 1, 1, 0, 1])  # Movement possibility in 8 directions
    robot_loc = [500, 500]  # Robot's location in the grid
    
    rate = rospy.Rate(1)  # Frequency of grid updates (1 Hz)
    center = [-50,-50]
    
    
    while not rospy.is_shutdown():
        robot_loc = [50, 50]
        # center = [500,500]
        # robot_loc[0] = robot_loc[0] + center[0]
        # robot_loc[1] = robot_loc[1] + center[1]
        
        
        
        
        
        
        grid1.data = mark_explored(obstacle, square_size, grid1.data, robot_loc, grid1.info.width, grid1.info.height)
        grid1.data = mark_obstacles(obstacle, square_size, grid1.data, robot_loc, grid1.info.width, grid1.info.height)
        
        grid2.data = mark_explored(obstacle, square_size, grid2.data, robot_loc, grid2.info.width, grid2.info.height)
        grid2.data = mark_obstacles(obstacle, square_size, grid2.data, robot_loc, grid2.info.width, grid2.info.height)
        
        grid3.data = mark_explored(obstacle, square_size, grid3.data, robot_loc, grid3.info.width, grid3.info.height)
        grid3.data = mark_obstacles(obstacle, square_size, grid3.data, robot_loc, grid3.info.width, grid3.info.height)
        
        grid1.header.stamp = rospy.Time.now()
        pub1.publish(grid1)
        
        grid2.header.stamp = rospy.Time.now()
        pub2.publish(grid2)
        
        grid3.header.stamp = rospy.Time.now()
        pub3.publish(grid3)

        rate.sleep()


if __name__ == '__main__':
    try:
        draw_ogm()
    except rospy.ROSInterruptException:
        pass
