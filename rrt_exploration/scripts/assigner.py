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
import pandas as pd

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
globalmaps=[]

previous_time = None
file_path = '/home/icon-group/catkin_ws/src/multi_robots/multirobot_with_llm/collect_data/baseline_results.csv'

def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
	global mapData,previous_time
	previous_time = time.time()
	mapData=data
	
# Node----------------------------------------------



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



def node():
	global frontiers,mapData,globalmaps,previous_time
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

	robots=[]
	for i in range(0,len(robot_namelist)):
		robots.append(robot(name=robot_namelist[i]))

 
 
	for i in range(0,len(robot_namelist)):
		robots[i].sendGoal(robots[i].getPosition())   
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		centroids=copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,len(robot_namelist)):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		
		for ir in na:
			for ip in range(0,len(centroids)):
				cost=norm(robots[ir].getPosition()-centroids[ip])		
				threshold=1
				information_gain=infoGain[ip]
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):

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
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		rospy.loginfo("revenue record: "+str(revenue_record))	
		rospy.loginfo("centroid record: "+str(centroid_record))	
		# rospy.loginfo("robot IDs record: "+str(id_record))	
		
#-------------------------------------------------------------------------	
		if (len(id_record)>0 and previous_time is not None):
			winner_id=revenue_record.index(max(revenue_record))
			
			if(robot_namelist[id_record[winner_id]]=='jb_0'):
				column_name = 'rrt_1'  
				currnt_t = time.time()
				new_value = str(currnt_t-previous_time)
				append_to_column(file_path, column_name, new_value)
				robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
				column_name = 'server_robot1'  
				currnt_t = time.time()
				new_value = str(currnt_t-previous_time)


				
			if(robot_namelist[id_record[winner_id]]=='jb_1'):
				column_name = 'rrt_2'  
				currnt_t = time.time()
				new_value = str(currnt_t-previous_time)
				append_to_column(file_path, column_name, new_value)
				robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
				column_name = 'server_robot2'  
				currnt_t = time.time()
				new_value = str(time.time)
				append_to_column(file_path, column_name, new_value)
	
			rospy.loginfo(robot_namelist[id_record[winner_id]] + "  assigned to  "+str(centroid_record[winner_id]))	
			rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
