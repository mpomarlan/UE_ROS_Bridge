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
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    msg.result.picture_taken = readBool(params['picture_taken'])
    msg.result.picture_hw_available = readBool(params['picture_hw_available'])
    msg.result.picture_id = str(params['picture_id'])
    return msg

def readTakePictureFeedback(params):
    msg = sherpa_msgs.msg.TakePictureActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    msg.feedback.picture_taken = readBool(params['picture_taken'])
    msg.feedback.picture_hw_available = readBool(params['picture_hw_available'])
    msg.feedback.picture_id = str(params['picture_id'])
    return msg

def readMoveToResult(params):
    msg = sherpa_msgs.msg.MoveToActionResult()
    print "readMoveToResult:"
    print params
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'moving' in params:
        msg.result.moving = readBool(params['moving'])
    else:
        msg.result.moving = false
    if 'reached' in params:
        msg.result.reached = readBool(params['reached'])
    else:
        msg.result.reached = false
    return msg

def readMoveToFeedback(params):
    msg = sherpa_msgs.msg.MoveToActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'moving' in params:
        msg.feedback.moving = readBool(params['moving'])
    else:
        msg.feedback.moving = false
    if 'reached' in params:
        msg.feedback.reached = readBool(params['reached'])
    else:
        msg.feedback.reached = false
    return msg

def readToggleActuatorResult(params):
    msg = sherpa_msgs.msg.ToggleActuatorActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    msg.result.state = readBool(params['state'])
    return msg

def readToggleActuatorFeedback(params):
    msg = sherpa_msgs.msg.ToggleActuatorActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
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
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    msg.feedback.motors_on = readBool(params['motors_on'])
    msg.feedback.motor_hw_available = readBool(params['motor_hw_available'])
    msg.feedback.reached = readBool(params['reached'])
    msg.feedback.current_altitude = float(params['current_altitude'])
    return msg

def readBeacon(params):
    print params
    msg = sherpa_msgs.msg.Beacon()
    msg.direction.x = float(params['x'])
    msg.direction.y = float(params['y'])
    msg.direction.z = float(params['z'])
    msg.beacon_value = float(params['beacon_value'])
    msg.robot_name = str(params['robot_name'])
    print "Msg"
    print msg
    return msg

def readBattery(params):
    msg = sherpa_msgs.msg.Battery()
    msg.battery_level = float(params['battery_level'])
    msg.battery_drain = float(params['battery_drain'])
    return msg

def readMountActionResult(params):
    msg = sherpa_msgs.msg.MountActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'target_found' in params:
        msg.result.target_found = readBool(params['target_found'])
    if 'space_available' in params:
        msg.result.target_found = readBool(params['space_available'])
    if 'mounting_hw_available' in params:
        msg.result.target_found = readBool(params['mounting_hw_available'])
    if 'manipulation_error' in params:
        msg.result.target_found = readBool(params['manipulation_error'])
    if 'mounted_target' in params:
        msg.result.target_found = readBool(params['mounted_target'])
    return msg

def readMountActionFeedback(params):
    msg = sherpa_msgs.msg.MountActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'target_found' in params:
        msg.feedback.target_found = readBool(params['target_found'])
    if 'space_available' in params:
        msg.feedback.target_found = readBool(params['space_available'])
    if 'mounting_hw_available' in params:
        msg.feedback.target_found = readBool(params['mounting_hw_available'])
    if 'manipulation_error' in params:
        msg.feedback.target_found = readBool(params['manipulation_error'])
    if 'mounted_target' in params:
        msg.feedback.target_found = readBool(params['mounted_target'])
    return msg

def readLogEventActionResult(params):
    msg = sherpa_msgs.msg.LogEventActionResult()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'logged' in params:
        msg.result.logged = readBool(params['logged'])
    return msg

def readLogEventActionFeedback(params):
    msg = sherpa_msgs.msg.LogEventActionFeedback()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    if 'rejected_goal_id' in params:
        msg.status = readGoalStatus(params, 'rejected_')
        return msg
    msg.status = readGoalStatus(params, '')
    if 'logged' in params:
        msg.feedback.logged = readBool(params['logged'])
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
                   ('red_wasp/beacon', sherpa_msgs.msg.Beacon, 1, readBeacon),
                   ('red_wasp/battery', sherpa_msgs.msg.Battery, 1, readBattery),
                   ('blue_wasp/TakePicture/result', sherpa_msgs.msg.TakePictureActionResult, 1, readTakePictureResult), 
                   ('blue_wasp/TakePicture/feedback', sherpa_msgs.msg.TakePictureActionFeedback, 1, readTakePictureFeedback),
                   ('blue_wasp/TakePicture/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('blue_wasp/Fly/result', sherpa_msgs.msg.MoveToActionResult, 1, readMoveToResult), 
                   ('blue_wasp/Fly/feedback', sherpa_msgs.msg.MoveToActionFeedback, 1, readMoveToFeedback),
                   ('blue_wasp/Fly/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('blue_wasp/ToggleEngine/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult), 
                   ('blue_wasp/ToggleEngine/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('blue_wasp/ToggleEngine/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('blue_wasp/SetAltitude/result', sherpa_msgs.msg.SetAltitudeActionResult, 1, readSetAltitudeResult),
                   ('blue_wasp/SetAltitude/feedback', sherpa_msgs.msg.SetAltitudeActionFeedback, 1, readSetAltitudeFeedback),
                   ('blue_wasp/SetAltitude/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('blue_wasp/ToggleBeacon/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult),
                   ('blue_wasp/ToggleBeacon/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('blue_wasp/ToggleBeacon/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('blue_wasp/beacon', sherpa_msgs.msg.Beacon, 1, readBeacon),
                   ('blue_wasp/battery', sherpa_msgs.msg.Battery, 1, readBattery),
                   ('hawk/TakePicture/result', sherpa_msgs.msg.TakePictureActionResult, 1, readTakePictureResult), 
                   ('hawk/TakePicture/feedback', sherpa_msgs.msg.TakePictureActionFeedback, 1, readTakePictureFeedback),
                   ('hawk/TakePicture/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('hawk/Fly/result', sherpa_msgs.msg.MoveToActionResult, 1, readMoveToResult), 
                   ('hawk/Fly/feedback', sherpa_msgs.msg.MoveToActionFeedback, 1, readMoveToFeedback),
                   ('hawk/Fly/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('hawk/ToggleEngine/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult), 
                   ('hawk/ToggleEngine/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('hawk/ToggleEngine/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('hawk/SetAltitude/result', sherpa_msgs.msg.SetAltitudeActionResult, 1, readSetAltitudeResult),
                   ('hawk/SetAltitude/feedback', sherpa_msgs.msg.SetAltitudeActionFeedback, 1, readSetAltitudeFeedback),
                   ('hawk/SetAltitude/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('hawk/ToggleBeacon/result', sherpa_msgs.msg.ToggleActuatorActionResult, 1, readToggleActuatorResult),
                   ('hawk/ToggleBeacon/feedback', sherpa_msgs.msg.ToggleActuatorActionFeedback, 1, readToggleActuatorFeedback),
                   ('hawk/ToggleBeacon/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('donkey/Drive/result', sherpa_msgs.msg.MoveToActionResult, 1, readMoveToResult), 
                   ('donkey/Drive/feedback', sherpa_msgs.msg.MoveToActionFeedback, 1, readMoveToFeedback),
                   ('donkey/Drive/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('donkey/Mount/result', sherpa_msgs.msg.MountActionResult, 1, readMountActionResult),
                   ('donkey/Mount/feedback', sherpa_msgs.msg.MountActionFeedback, 1, readMountActionFeedback),
                   ('donkey/Mount/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray),
                   ('ue_semlog/LogEvent/result', sherpa_msgs.msg.LogEventActionResult, 1, readLogEventActionResult),
                   ('ue_semlog/LogEvent/feedback', sherpa_msgs.msg.LogEventActionFeedback, 1, readLogEventActionFeedback),
                   ('ue_semlog/LogEvent/status', actionlib_msgs.msg.GoalStatusArray, 1, readGoalStatusArray))

