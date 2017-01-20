#! /usr/bin/env python

import rospy

import time

import sys
import signal


# Brings in the SimpleActionClient
import actionlib
import actionlib_msgs.msg

import sherpa_msgs.msg

client = actionlib.SimpleActionClient('red_wasp/TakePicture', sherpa_msgs.msg.TakePictureAction)

def exit_gracefully(signum, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, exit_gracefully)
signal.signal(signal.SIGTERM, exit_gracefully)

rospy.init_node('takepicture_client_py')

client.wait_for_server(rospy.Duration(2.0))
print "Server for red_wasp/TakePicture has started, could try sending a goal."

print client.action_client.feedback_sub.impl.get_stats()
print client.action_client.result_sub.impl.get_stats()
print client.action_client.status_sub.impl.get_stats()

# Creates a goal to send to the action server.
goal = sherpa_msgs.msg.TakePictureActionGoal()

# Sends the goal to the action server.
client.send_goal(goal)

# Waits for the server to finish performing the action.
#client.wait_for_result()

# Prints out the result of executing the action
#print client.get_result()  # A TakePictureActionResult

while True:
    # Waits until the action server has started up and started
    # listening for goals.
#    # Creates a goal to send to the action server.
#    goal = sherpa_msgs.msg.TakePictureActionGoal()

#    # Sends the goal to the action server.
    client.send_goal(goal)

#    # Waits for the server to finish performing the action.
#    time.sleep(1)
    client.wait_for_result()

#    # Prints out the result of executing the action
    print client.get_result()  # A TakePictureActionResult
