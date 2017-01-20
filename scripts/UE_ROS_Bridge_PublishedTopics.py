import rospy
from std_msgs.msg import String

import sherpa_msgs.msg
import actionlib_msgs.msg

def readBool(strVal):
    boolVal = True
    if ('0' == str(strVal)) or ('false' == str(strVal).lower()):
        boolVal = False
    return boolVal

def readGoalId(params, prefix):
    goalId = actionlib_msgs.msg.GoalID()
    goalId.id = str(params[prefix + 'goal_id'])
    # TODO: get stamp from a goal_stamp member, if it exists
    goalId.stamp = rospy.Time.now()
    return goalId

def readGoalStatus(params, prefix):
    goalStatus = actionlib_msgs.msg.GoalStatus()
    goalStatus.goal_id = readGoalId(params, prefix)
    goalStatus.status = int(params[prefix + 'goal_status'])
    textKey = prefix + 'text'
    if textKey in params:
        goalStatus.text = str(params[textKey])
    return goalStatus

def readGoalStatusArray(params):
    msg = actionlib_msgs.msg.GoalStatusArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    goalMsg = readGoalStatus(params, '')
    msg.status_list.append(goalMsg)
    if ('rejected_goal_id' in params) and ('rejected_goal_status' in params):
        rejMsg = readGoalStatus(params, 'rejected_')
        msg.status_list.append(rejMsg)
    return msg

def readTakePictureResult(params):
    msg = sherpa_msgs.msg.TakePictureActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.result.picture_taken = readBool(params['picture_taken'])
    msg.result.picture_hw_available = readBool(params['picture_hw_available'])
    msg.result.picture_id = str(params['picture_id'])
    return msg

def readTakePictureFeedback(params):
    msg = sherpa_msgs.msg.TakePictureActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.feedback.picture_taken = readBool(params['picture_taken'])
    msg.feedback.picture_hw_available = readBool(params['picture_hw_available'])
    msg.feedback.picture_id = str(params['picture_id'])
    return msg

def readMoveToResult(params):
    msg = sherpa_msgs.msg.MoveToActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.result.moving = readBool(params['moving'])
    msg.result.reached = readBool(params['reached'])
    return msg

def readMoveToFeedback(params):
    msg = sherpa_msgs.msg.MoveToActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.feedback.moving = readBool(params['moving'])
    msg.feedback.reached = readBool(params['reached'])
    return msg

PublishedTopics = (('red_wasp/TakePicture/result', sherpa_msgs.msg.TakePictureActionResult, 1, readTakePictureResult), 
                   ('red_wasp/TakePicture/feedback', sherpa_msgs.msg.TakePictureActionFeedback, 1, readTakePictureFeedback),
                   ('red_wasp/TakePicture/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/Fly/result', sherpa_msgs.msg.MoveToActionResult, 1, readMoveToResult), 
                   ('red_wasp/Fly/feedback', sherpa_msgs.msg.MoveToActionFeedback, 1, readMoveToFeedback),
                   ('red_wasp/Fly/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray))

