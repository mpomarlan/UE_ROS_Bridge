import rospy
from std_msgs.msg import String

def readString(data):
    return {"data": data.data}

def readDisplayText(msg):
    return {"text": msg.data}
	
SubscribedTopics = (('test_topic', String, readString), ('display_command', String, readDisplayText))
