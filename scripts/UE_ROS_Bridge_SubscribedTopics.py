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
    print str({"goalId": str(msg.goal_id.id), "command": bool2Str(msg.goal.command)})
    return {"goalId": str(msg.goal_id.id), "command": bool2Str(msg.goal.command)}

def readSetAltitudeActionGoal(msg):
    return {"goalId": str(msg.goal_id.id), "altitude": str(msg.goal.altitude)}

def readMountActionGoal(msg):
    print "Donkey"
    print {"goalId": str(msg.goal_id.id), "target_name": str(msg.goal.target_name), "mounted_state": bool2Str(msg.goal.mounted_state)}
    return {"goalId": str(msg.goal_id.id), "target_name": str(msg.goal.target_name), "mounted_state": bool2Str(msg.goal.mounted_state)}

def LoggedRDFEntry2Str(msg):
    return str(msg.property_name) + "|" + str(msg.rdf_datatype) + "|" + str(msg.value) + "|" + str(msg.rdf_resource) + "|" + bool2Str(msg.use_resource)

def readLogEventActionGoal(msg):
    rdf_entries = ""
    for rdf_entry in msg.goal.rdf_entries:
        rdf_entries = rdf_entries + "@" + LoggedRDFEntry2Str(rdf_entry)
    return {"goalId": str(msg.goal_id.id),
            "name": str(msg.goal.name),
            "name_id_flag": str(msg.goal.name_id_flag),
            "type": str(msg.goal.type),
            "rdf_entries": rdf_entries}

SubscribedTopics = (('test_topic', String, readString), ('display_command', String, readDisplayText), 
                    ('red_wasp/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('red_wasp/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('red_wasp/Fly/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/ToggleEngine/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('red_wasp/ToggleEngine/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/SetAltitude/goal', sherpa_msgs.msg.SetAltitudeActionGoal, readSetAltitudeActionGoal), ('red_wasp/SetAltitude/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('red_wasp/ToggleBeacon/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('red_wasp/ToggleBeacon/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('blue_wasp/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('blue_wasp/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('blue_wasp/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('blue_wasp/Fly/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('blue_wasp/ToggleEngine/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('blue_wasp/ToggleEngine/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('blue_wasp/SetAltitude/goal', sherpa_msgs.msg.SetAltitudeActionGoal, readSetAltitudeActionGoal), ('blue_wasp/SetAltitude/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('blue_wasp/ToggleBeacon/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('blue_wasp/ToggleBeacon/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('hawk/TakePicture/goal', sherpa_msgs.msg.TakePictureActionGoal, readTakePictureGoal), ('hawk/TakePicture/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('hawk/Fly/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('hawk/Fly/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('hawk/ToggleEngine/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('hawk/ToggleEngine/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('hawk/SetAltitude/goal', sherpa_msgs.msg.SetAltitudeActionGoal, readSetAltitudeActionGoal), ('hawk/SetAltitude/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('hawk/ToggleBeacon/goal', sherpa_msgs.msg.ToggleActuatorActionGoal, readToggleActuatorActionGoal), ('hawk/ToggleBeacon/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('donkey/Mount/goal', sherpa_msgs.msg.MountActionGoal, readMountActionGoal), ('donkey/Mount/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('donkey/Drive/goal', sherpa_msgs.msg.MoveToActionGoal, readMoveToGoal), ('donkey/Drive/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel),
                    ('ue_semlog/LogEvent/goal', sherpa_msgs.msg.LogEventActionGoal, readLogEventActionGoal), ('ue_semlog/LogEvent/cancel', actionlib_msgs.msg.GoalID, readActionlibCancel))

