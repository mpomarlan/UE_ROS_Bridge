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
    if ('goal_id' in params) and ('goal_status' in params):
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
    print "FLY"
    print params
    msg = sherpa_msgs.msg.MoveToActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    print "."
    msg.result.moving = readBool(params['moving'])
    msg.result.reached = readBool(params['reached'])
    print msg
    return msg

def readMoveToFeedback(params):
    msg = sherpa_msgs.msg.MoveToActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.feedback.moving = readBool(params['moving'])
    msg.feedback.reached = readBool(params['reached'])
    return msg

def readToggleActuatorResult(params):
    msg = sherpa_msgs.msg.ToggleActuatorActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.result.state = readBool(params['state'])
    return msg

def readToggleActuatorFeedback(params):
    msg = sherpa_msgs.msg.ToggleActuatorActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.feedback.state = readBool(params['state'])
    return msg

def readSetAltitudeResult(params):
    msg = sherpa_msgs.msg.SetAltitudeActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    msg.result.motors_on = readBool(params['motors_on'])
    msg.result.motor_hw_available = readBool(params['motor_hw_available'])
    msg.result.reached = readBool(params['reached'])
    msg.result.current_altitude = float(params['current_altitude'])
    return msg

def readSetAltitudeFeedback(params):
    msg = sherpa_msgs.msg.SetAltitudeActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.status = readGoalStatus(params, '')
    msg.feedback.motors_on = readBool(params['motors_on'])
    msg.feedback.motor_hw_available = readBool(params['motor_hw_available'])
    msg.feedback.reached = readBool(params['reached'])
    msg.feedback.current_altitude = float(params['current_altitude'])
    return msg

def readBeacon(params):
    msg = sherpa_msgs.msg.Beacon()
    msg.direction.x = float(params['x'])
    msg.direction.y = float(params['y'])
    msg.direction.z = float(params['z'])
    msg.beacon_value = float(params['beacon_value'])
    msg.robot_name = str(params['robot_name'])
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    return msg

PublishedTopics = (('red_wasp/TakePicture/result', sherpa_msgs.msg.TakePictureActionResult, 1, readTakePictureResult), 
                   ('red_wasp/TakePicture/feedback', sherpa_msgs.msg.TakePictureActionFeedback, 1, readTakePictureFeedback),
                   ('red_wasp/TakePicture/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/Fly/result', sherpa_msgs.msg.MoveToActionResult, 1, readMoveToResult), 
                   ('red_wasp/Fly/feedback', sherpa_msgs.msg.MoveToActionFeedback, 1, readMoveToFeedback),
                   ('red_wasp/Fly/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/ToggleEngine/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult), 
                   ('red_wasp/ToggleEngine/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('red_wasp/ToggleEngine/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/SetAltitude/result', sherpa_msgs.msg.SetAltitudeActionResult, 1, readSetAltitudeResult),
                   ('red_wasp/SetAltitude/feedback', sherpa_msgs.msg.SetAltitudeActionFeedback, 1, readSetAltitudeFeedback),
                   ('red_wasp/SetAltitude/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/ToggleBeacon/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult),
                   ('red_wasp/ToggleBeacon/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('red_wasp/ToggleBeacon/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('red_wasp/beacon', sherpa_msgs.msg.Beacon, 1, readBeacon))

