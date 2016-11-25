import rospy

#Replace these with command line params
from UE_ROS_Bridge_SubscribedTopics import SubscribedTopics
from UE_ROS_Bridge_UsedServices import UsedServices

def basicCallback(mutex, messagePages, messageSendingPage, topicName, newData):
    mutex.acquire()
    messageWritingPage = messageSendingPage[0] ^ 1
    messagePages[messageWritingPage][topicName] = newData
    mutex.release()

def makeCallback(mutex, messagePages, messageSendingPage, topicName, dataReader):
    def callback(data):
	    basicCallback(mutex, messagePages, messageSendingPage, topicName, dataReader(data))
    return callback

def makeServiceCaller(serviceHandle, mutex, messagePages, messageSendingPage, name, readQuery, readResponse):
    responseKey = name + "_response"
    def caller(data):
        ROSRequest = readQuery(data)
        response = readResponse(serviceHandle(ROSRequest))
        mutex.acquire()
        messageWritingPage = messageSendingPage[0] ^ 1
        messagePages[messageWritingPage][responseKey] = response
        mutex.release()
    return caller

def SetupListeners(mutex, messagePages, messageSendingPage):
    for topic in SubscribedTopics:
	    #                name      type                                                                      msg reader
	    rospy.Subscriber(topic[0], topic[1], makeCallback(mutex, messagePages, messageSendingPage, topic[0], topic[2]))


def SetupServiceListeners(mutex, messagePages, messageSendingPage, serviceHandlers):
    for service in UsedServices:
        #                                  name        type
        serviceHandle = rospy.ServiceProxy(service[0], service[1])
                                      #                                                                         name        readQuery   readResponse
        serviceHandlers[service[0]] = makeServiceCaller(serviceHandle, mutex, messagePages, messageSendingPage, service[0], service[2], service[3]))
        
