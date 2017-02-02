import rospy
from std_msgs.msg import String

import sherpa_msgs.msg
import actionlib_msgs.msg

def bool2Str(bVar):
    if bVar:
        return "1"
    return "0"

def readString(data):
    return {"data": str(data.data)}

def readActionlibCancel(msg):
    return {"goalId": str(msg.id)}
	
def readDisplayText(msg):
    return {"text": str(msg.data)}

def readTakePictureGoal(msg):
    #print {"goalId": str(msg.goal_id.id)}
    return {"goalId": str(msg.goal_id.id), "picture_id": str(msg.goal.picture_id)}

def readMoveToGoal(msg):
    return {"goalId": str(msg.goal_id.id), "frameId": str(msg.goal.goal.header.frame_id), 
            "x": str(msg.goal.goal.pose.position.x), "y": str(msg.goal.goal.pose.position.y), "z": str(msg.goal.goal.pose.position.z), 
            "qx": str(msg.goal.goal.pose.orientation.x), "qy": str(msg.goal.goal.pose.orientation.y), "qz": str(msg.goal.goal.pose.orientation.z), "qw": str(msg.goal.goal.pose.orientation.w)}

def readToggleActuatorActionGoal(msg):
    print "TOGGLE GOAL"
    print str({"goalId": str(msg.goal_id.id), "command": bool2Str(msg.goal.command)})
    return {"goalId": str(msg.goal_id.id), "command": bool2Str(msg.goal.command)}

def readSetAltitudeActionGoal(msg):
    return {"goalId": str(msg.goal_id.id), "altitude": str(msg.goal.altitude)}

SubscribedTopics = (('test_topic', String, readString), ('display_command', String, readDisplayText), 
                    ('red_wasp/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('red_wasp/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('red_wasp/Fly/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/ToggleEngine/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('red_wasp/ToggleEngine/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/SetAltitude/goal', sherpa_msgs.msg.SetAltitudeActionGoal, readSetAltitudeActionGoal), ('red_wasp/SetAltitude/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/ToggleBeacon/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('red_wasp/ToggleBeacon/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel))

