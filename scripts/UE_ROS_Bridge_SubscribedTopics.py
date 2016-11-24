import rospy
from std_msgs.msg import String

def readString(data):
    return {"data": data.data}
	
SubscribedTopics = (('test_topic', String, readString))
