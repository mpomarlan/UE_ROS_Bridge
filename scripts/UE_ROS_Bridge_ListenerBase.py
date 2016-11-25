import rospy

#Replace this with command line params
from UE_ROS_Bridge_SubscribedTopics import SubscribedTopics

def basicCallback(mutex, messagePages, messageSendingPage, topicName, newData):
    mutex.acquire()
    messageWritingPage = messageSendingPage[0] ^ 1
    messagePages[messageWritingPage][topicName] = newData
    mutex.release()

def makeCallback(mutex, messagePages, messageSendingPage, topicName, dataReader):
    def callback(data):
	    basicCallback(mutex, messagePages, messageSendingPage, topicName, dataReader(data))
    return callback

def SetupListeners(mutex, messagePages, messageSendingPage):
    for topic in SubscribedTopics:
	    #                name      type                                                                      msg reader
	    rospy.Subscriber(topic[0], topic[1], makeCallback(mutex, messagePages, messageSendingPage, topic[0], topic[2]))
