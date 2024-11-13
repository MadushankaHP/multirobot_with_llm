#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from rospy import Time

def map_callback(data):
    # Get the publish timestamp from the map header (when the map was published)
    publish_time = data.header.stamp
    
    # Get the current time (when the map is received by the subscriber)
    subscribe_time = rospy.Time.now()

    # Calculate the time difference in seconds
    time_difference = (subscribe_time - publish_time).to_sec()

    rospy.loginfo(f"Time difference between map publish and subscribe: {time_difference:.6f} seconds")

# Initialize the ROS node
rospy.init_node('map_publish_subscribe_difference', anonymous=True)

# Subscribe to the /map topic
rospy.Subscriber("/jb_1/map", OccupancyGrid, map_callback)

# Keep the node running
rospy.loginfo("Node initialized and waiting for /map messages...")
rospy.spin()
