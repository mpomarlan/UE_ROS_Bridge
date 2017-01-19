import rospy
from std_msgs.msg import String

import sherpa_msgs.msg
import actionlib_msgs.msg

def readBool(strVal):
    boolVal = True
    if ('0' == strVal) or ('false' == strVal.lower()):
        boolVal = False
    return boolVal

def readGoalStatusArray(params):
    msg = actionlib_msgs.msg.GoalStatusArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    goalMsg = actionlib_msgs.msg.GoalStatus()
    goalMsg.goal_id.stamp = rospy.Time.now()
    goalMsg.goal_id.id = params['goal_id']
    goalMsg.status = int(params['status'])
    if 'text' in params:
        goalMsg.status.text = params['text']
    msg.header.status_list.append(goalMsg)
    return msg

def readTakePictureResult(params):
    msg = sherpa_msgs.msg.TakePictureActionResult()
    msg.picture_taken = readBool(params['picture_taken'])
    msg.picture_hw_available = readBool(params['picture_hw_available'])
    msg.picture_id = readBool(params['picture_id'])
    return msg

def readTakePictureFeedback(params):
    msg = sherpa_msgs.msg.TakePictureActionFeedback()
    msg.picture_taken = readBool(params['picture_taken'])
    msg.picture_hw_available = readBool(params['picture_hw_available'])
    msg.picture_id = readBool(params['picture_id'])
    return msg

PublishedTopics = (('red_wasp/TakePicture/result', sherpa_msgs.msg.TakePictureActionResult, 1, readTakePictureResult), 
                   ('red_wasp/TakePicture/feedback', sherpa_msgs.msg.TakePictureActionFeedback, 1, readTakePictureFeedback),
                   ('red_wasp/TakePicture/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray))
