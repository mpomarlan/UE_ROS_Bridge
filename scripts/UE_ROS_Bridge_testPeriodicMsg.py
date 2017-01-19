import time

import rospy
from std_msgs.msg import String
from threading import Thread


topicName = 'test_topic'
topicName = 'display_command'
pub = rospy.Publisher(topicName, String, queue_size=1)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
k = 0

while not rospy.is_shutdown():
    tN = rospy.Time.now()
    pub.publish("Display Text: " + str(k) + " :: " + str(tN.secs) + " " + str(tN.nsecs))
    k = k + 1
    r.sleep()

