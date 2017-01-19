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

SubscribedTopics = (('test_topic', String, readString), ('display_command', String, readDisplayText), 
                    ('red_wasp/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('red_wasp/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel))
