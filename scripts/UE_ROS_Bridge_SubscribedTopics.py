import rospy
from std_msgs.msg import String

import sherpa_msgs.msg
import actionlib_msgs.msg

def readString(data):
    return {"data": str(data.data)}

def readActionlibCancel(msg):
    return {"goal_id": str(msg.id)}
	
def readDisplayText(msg):
    return {"text": str(msg.data)}

def readTakePictureGoal(msg):
    return {"goal_id": str(msg.goal_id.id)}

def readMoveToGoal(msg):
    return {"goal_id": str(msg.goal_id.id), "frame_id": msg.goal.header.frame_id, 
            "x": str(msg.goal.pose.position.x), "y": str(msg.goal.pose.position.y), "z": str(msg.goal.pose.position.z), 
            "qx": str(msg.goal.pose.orientation.x), "qy": str(msg.goal.pose.orientation.y), "qz": str(msg.goal.pose.orientation.z), "qw": str(msg.goal.pose.orientation.w)}

SubscribedTopics = (('test_topic', String, readString), ('display_command', String, readDisplayText), 
                    ('red_wasp/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('red_wasp/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('red_wasp/Fly/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel))
