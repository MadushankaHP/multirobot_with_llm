import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('map_timestamp_difference', anonymous=True)

previous_time = None

def map_callback(data):
    global previous_time  # Use the global variable for previous_time

    current_time = data.header.stamp

    if previous_time is not None:
        time_difference = (current_time - previous_time).to_sec()
        rospy.loginfo(f"Time difference between previous and current map: {time_difference:.6f} seconds")

    previous_time = current_time

rospy.Subscriber("/jb_1/map", OccupancyGrid, map_callback)

rospy.loginfo("Node initialized. Waiting for map messages...")

# Keep the node running
rospy.spin()