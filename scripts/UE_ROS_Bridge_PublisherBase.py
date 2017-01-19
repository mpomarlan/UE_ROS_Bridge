import rospy

#Replace these with command line params
from UE_ROS_Bridge_PublishedTopics import PublishedTopics

def SetupPublishers(publisherMap):
    for topic in PublishedTopics:
	    #            name                         name      type                 queue_size reader
            publisherMap[topic[0]] = (rospy.Publisher(topic[0], topic[1], queue_size=topic[2]), topic[3])
        
