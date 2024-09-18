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



rbot1_ldr = None
loc = None
robot1_status = 10

def callback1(message1: LaserScan):
    global rbot1_ldr
    lidar = np.array(message1.ranges)
    #rbot1_ldr = message1.ranges
    lidar[np.isinf(lidar)] = -1 
    rbot1_ldr = np.append(lidar, -1)
    # save_data(rbot1_ldr, 'data/lidar_data.npy')


def callback2(message1: Odometry):
    global loc
   # print(message1.pose.pose.position)
    # save_data(np.array(message1.pose.pose.position), 'data/imu_data.npy')
    
    loc = 'robot loc : (' + str(round(message1.pose.pose.position.x,2)) + ',' + str(round(message1.pose.pose.position.y,2))   +')'
    
    
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

def main():
    
    global rbot1_ldr, rbot2_ldr,loc
    global robot1_status

   
    topic1 = '/jb_0/scan';
    topic2 = '/jb_0/odom';
    topic4 = '/jb_0/move_base/status'
    
    rospy.init_node('R1_scan_data',anonymous=True);

    rospy.Subscriber(topic1,LaserScan, callback1);
    rospy.Subscriber(topic2,Odometry, callback2);
    rospy.Subscriber(topic4,GoalStatusArray, callback3);

    pub = rospy.Publisher('/obstacle_array', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/jb_0/text_des', String, queue_size=10)

    cor = True
    flag_1 = True
    while not rospy.is_shutdown():
        
        
        if rbot1_ldr is not None and loc is not None:
            
            if(flag_1==True or robot1_status==3):
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
                    
                    if(count>35):
                        obstcle[i] = 0
                        
                    elif(dis_avg<2):
                        obstcle[i] = 1
                        
                array_msg = Float64MultiArray()
                array_msg.data = obstcle
                pub.publish(array_msg)
                
                lidar_data = 'Lidar data : (' + str(obstcle[0]) +','+ str(obstcle[1]) +','+ str(obstcle[2]) +','+ str(obstcle[3]) +','+ str(obstcle[4]) +','+ str(obstcle[5]) +','+ str(obstcle[6]) +','+ str(obstcle[7]) +')'
                # print(loc)
                # print(lidar_data)
                
                in_str = loc + " " + lidar_data
                
                if((cor ==True and robot1_status==3) or flag_1):
                    output = {
                    "coordinates": loc,
                    "possible_directions": lidar_data
                    }
                    print(output)
                    # out = infer(in_str)
                    for i in range(50):
                        print("publish robot 1 textual description")
                        pub2.publish(str(output))
                    cor = False
                    flag_1 = False
                    
            elif(robot1_status!=3):
                cor = True
            
        
            

                    
                        
            
def save_data(data, filename):
    """Save numpy array data to a .npy file."""
    np.save(filename, data)


if __name__ == '__main__':
    
    try:
        main()
       
    except rospy.ROSInterruptException:
        rospy.logerr('Subscriber is error');
        pass;