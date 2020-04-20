#!/usr/bin/env python

# Dummy class used as an example of how to do inter-node communication

import rospy
from std_msgs.msg import String

# Initialize the node with rospy
rospy.init_node('publisher_node')
# Create publisher
publisher = rospy.Publisher("~topic", String, queue_size=1)


# Define Timer callback
def callback(event):
    msg = String()
    publisher.publish("hi")


# Read parameter
pub_period = rospy.get_param("~pub_period", 1.0)
# Create timer
rospy.Timer(rospy.Duration.from_sec(pub_period), callback)
# spin to keep the script for exiting
rospy.spin()
