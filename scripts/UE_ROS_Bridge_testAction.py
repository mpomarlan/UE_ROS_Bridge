#!/usr/bin/env python

import rospy

import time

import sys
import signal

import string
import random


# Brings in the SimpleActionClient
import actionlib
import actionlib_msgs.msg

import sherpa_msgs.msg

client = actionlib.SimpleActionClient('red_wasp/TakePicture', sherpa_msgs.msg.TakePictureAction)
pubTE = rospy.Publisher('red_wasp/ToggleEngine/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, queue_size=1)
pubSA = rospy.Publisher('red_wasp/SetAltitude/goal', sherpa_msgs.msg.SetAltitudeActionGoal, queue_size=1)
pubF = rospy.Publisher('red_wasp/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, queue_size=1)

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
#goal = sherpa_msgs.msg.TakePictureActionGoal()
goalSA = sherpa_msgs.msg.SetAltitudeActionGoal()
goalTE = sherpa_msgs.msg.ToggleActuatorActionGoal()
goalF = sherpa_msgs.msg.MoveToActionGoal()
#goal.goal_id.id = "/cram_hl_123"
goalSA.goal_id.id = "/cram_hl_641088_1485424303_GOAL_14854246362922590000"
goalSA.goal.altitude = 1.0
goalTE.goal_id.id="".join(random.SystemRandom().choice(string.ascii_uppercase + string.ascii_lowercase + string.digits) for _ in range(8))
goalTE.goal.command = True
goalF.goal_id.id = "/cram_more_stuff"
goalF.goal.goal.pose.position.x = 581.0
goalF.goal.goal.pose.position.y = -298.0
goalF.goal.goal.pose.position.z = 0.0
goalF.goal.goal.pose.orientation.w = 1.0
print goalTE
#print goalSA
#print goalF

# Sends the goal to the action server.
#client.send_goal(goal)
pubTE.publish(goalTE)
#time.sleep(2)
#pubSA.publish(goalSA)
#time.sleep(5)
#pubF.publish(goalF)



from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:10090/')
)

remote_server = rpc_client.get_proxy()

msgs = []

#result = remote_server.ROSPublishTopics([])
#print result

# Waits for the server to finish performing the action.
#client.wait_for_result()

# Prints out the result of executing the action
#print client.get_result()  # A TakePictureActionResult

#while True:
    # Waits until the action server has started up and started
    # listening for goals.
#    # Creates a goal to send to the action server.
#    goal = sherpa_msgs.msg.TakePictureActionGoal()

#    # Sends the goal to the action server.
#    client.send_goal(goal)

#    # Waits for the server to finish performing the action.
#    time.sleep(1)
#    client.wait_for_result()

#    # Prints out the result of executing the action
#    print client.get_result()  # A TakePictureActionResult
