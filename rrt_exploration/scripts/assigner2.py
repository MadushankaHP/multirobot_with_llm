#!/usr/bin/env python3

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
# from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm
from geometry_msgs.msg import PoseStamped
import time
import math
from std_msgs.msg import String
import pandas as pd

file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/results.csv'

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]

check_loop1 = True
check_loop2 = True

check_status1 = True
check_status2 = True
previous_time = None

def callback1(data):
    global text_des_1,check_status1
    text_des_1 = data.data
    check_status1 = True

    
def callback2(data):
    global text_des_2,check_status2
    text_des_2 = data.data
    check_status2 = True




def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------
def getPosition(global_frame,robot_frame):
    listener = tf.TransformListener()

    cond = 0
    while cond == 0:
        try:
            (trans, rot) = listener.lookupTransform(
                global_frame, robot_frame, rospy.Time(0))
            cond = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            cond == 0
    position = array([trans[0], trans[1]])
    # print("************")
    # print(position)
    # print("------------")
    return position

def calculate_angle_and_direction(current_position, target_position):
    x1, y1 = current_position
    x2, y2 = target_position

    # Calculate differences in coordinates
    dx = x2*10 - x1
    dy = y2*10 - y1

    # Calculate angle in radians using atan2
    angle_rad = math.atan2(dy, dx)

    # Convert radians to degrees for easier interpretation
    angle_deg = math.degrees(angle_rad)

    # Normalize the angle to be between 0 and 360 degrees
    if angle_deg < 0:
        angle_deg += 360

    # Determine direction based on angle
    if 22.5 <= angle_deg < 67.5:
        direction = "northeast"
    elif 67.5 <= angle_deg < 112.5:
        direction = "north"
    elif 112.5 <= angle_deg < 157.5:
        direction = "northwest"
    elif 157.5 <= angle_deg < 202.5:
        direction = "west"
    elif 202.5 <= angle_deg < 247.5:
        direction = "southwest"
    elif 247.5 <= angle_deg < 292.5:
        direction = "south"
    elif 292.5 <= angle_deg < 337.5:
        direction = "southeast"
    else:
        direction = "east"

    # Calculate distance to the target position
    distance = math.sqrt(dx**2 + dy**2)

    return angle_deg, distance, direction

def save_data(file_path,data):
	
    with open(file_path, 'r') as file:
        content = file.readlines()

    for line in content:
        print(line.strip())
	
    with open(file_path, 'a') as file:
        file.write(data+"\n")

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





# Callback function for the /map topic
def map_callback(data):
    global previous_time  # Use the global variable for previous_time

    # Get the current timestamp from the map header
    previous_time = time.time()


def node():
	global frontiers,mapData,globalmaps
	global check_status1,check_status2 ,previous_time
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic								= rospy.get_param('~map_topic','/map')
	info_radius							= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier					= rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius				= rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain					= rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic					= rospy.get_param('~frontiers_topic','/filtered_points')	
	delay_after_assignement	= rospy.get_param('~delay_after_assignement',0.5)
	rateHz 									= rospy.get_param('~rate',100)
	robot_namelist          = rospy.get_param('~robot_namelist', "robot1")
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
 
	rospy.Subscriber('/jb_0/text_des', String, callback1)
	rospy.Subscriber('/jb_1/text_des', String, callback2)
	rospy.Subscriber("/global_map", OccupancyGrid, map_callback)

 
	pub_direction1 = rospy.Publisher('/jb_0/direction', String, queue_size=10)
	pub_direction2 = rospy.Publisher('/jb_1/direction', String, queue_size=10)
  
#---------------------------------------------------------------------------------------------------------------
	# perform name splitting for the robot
	robot_namelist = robot_namelist.split(',')
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	# robots=[]
	# for i in range(0,len(robot_namelist)):
	# 	robots.append(robot(name=robot_namelist[i]))

 
	# for i in range(0,len(robot_namelist)):
	# 	robots[i].sendGoal(robots[i].getPosition()) 
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	check_loop1 = True
	check_loop2 = True
	
	while not rospy.is_shutdown():
		global previous_time
		start_time = time.time()
     
		centroids=copy(frontiers)	
		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
		
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[0,1] #available robots
		nb=[] #busy robots
		# print(robots[0].getPosition())
		# print("**********")# 	if (robots[i].getState()==1):
		robot_pose = []		#nb.append(i)
		robot_pose.append(getPosition("global","robot_1/base_link"))# 	else:
		robot_pose.append(getPosition("global","robot_2/base_link"))# # 		na.append(i)	
		# rospy.loginfo("available robots: "+str(nb))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		
	
		for i in nb+na:
			infoGain=discount(mapData,robot_pose[i],centroids,infoGain,info_radius)

#-------------------------------------------------------------------------           
		revenue_record=[]
		centroid_record=[]
		id_record=[]
        
		#robot_pose = getPosition("map","jb_0/base_link")
		for ir in na:
			for ip in range(0,len(centroids)):
				cost=norm(robot_pose[ir]-centroids[ip])		
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robot_pose[ir]-centroids[ip])<=hysteresis_radius):

					information_gain*=hysteresis_gain
				revenue=information_gain*info_multiplier-cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
		
		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					cost=norm(robot_pose[ir]-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robot_pose[ir]-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robot_pose[i]))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		# rospy.loginfo("revenue record: "+str(revenue_record))	
		# rospy.loginfo("centroid record: "+str(centroid_record))	
		# rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			# robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			# target_goal = PoseStamped()
			# point = centroid_record[winner_id]
			if(check_status1 == True and robot_namelist[id_record[winner_id]]=='robot_1'):
				print(centroid_record[winner_id])
				print("***************")
				#direction1 = 1
				angle, distance, direction1 = calculate_angle_and_direction(robot_pose[0], centroid_record[winner_id])
				check_loop1 = False
				direction1 = direction1.lower()
                        
				
                        
				column_name = 'rrt_rbt1'  
				currnt_t = time.time()
				new_value = str(currnt_t-previous_time)
				append_to_column(file_path, column_name, new_value)

				column_name = 'server_2_rbt1'  
				currnt_t = time.time()
				new_value = str(currnt_t)
				append_to_column(file_path, column_name, new_value)
                        
				for i in range(30):
					pub_direction1.publish(direction1)
				print("*********")
				print(direction1)
				print("*********")
				check_status1 = False
				rospy.loginfo(robot_namelist[id_record[winner_id]] + "  assigned to  "+direction1)	
				rospy.sleep(delay_after_assignement)
    
			
				
			if(check_status2 == True and robot_namelist[id_record[winner_id]]=='robot_2'):
				print(centroid_record[winner_id])
				print("-----------------")
				#direction2 = 2
				angle, distance, direction2 = calculate_angle_and_direction(robot_pose[1], centroid_record[winner_id])
				check_loop2 = False
				direction2 = direction2.lower()
                        
                        
				column_name = 'rrt_rbt2'  
				new_value = str(time.time()-previous_time)
				append_to_column(file_path, column_name, new_value)
                        
				column_name = 'server_2_rbt2'  
				currnt_t = time.time()
				new_value = str(currnt_t)
				append_to_column(file_path, column_name, new_value)

#
				for i in range(30):
					pub_direction2.publish(direction2)
    
				print("---------")
				print(direction2)
				print("----------")
				check_status2 = False
			
   
				rospy.loginfo(robot_namelist[id_record[winner_id]] + "  assigned to  "+ direction2)	
				rospy.sleep(delay_after_assignement)
   

		
		elif(check_loop1 == False and check_loop2 == False):
			check_loop1 = True
			check_loop2 = True
                  
      
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
