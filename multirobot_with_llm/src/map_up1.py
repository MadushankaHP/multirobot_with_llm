import rospy
from nav_msgs.msg import OccupancyGrid

# Initialize the ROS node
rospy.init_node('map_timestamp_difference', anonymous=True)

# Initialize the previous timestamp to None
previous_time = None

# Callback function for the /map topic
def map_callback(data):
    global previous_time  # Use the global variable for previous_time

    # Get the current timestamp from the map header
    current_time = data.header.stamp

    if previous_time is not None:
        # Calculate the time difference in seconds
        time_difference = (current_time - previous_time).to_sec()
        rospy.loginfo(f"Time difference between previous and current map: {time_difference:.6f} seconds")

    # Update the previous timestamp for the next callback
    previous_time = current_time

# Subscribe to the /map topic
rospy.Subscriber("/jb_0/map", OccupancyGrid, map_callback)

rospy.loginfo("Node initialized. Waiting for map messages...")

# Keep the node running
rospy.spin()