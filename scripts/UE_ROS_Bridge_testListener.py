import rospy
from std_msgs.msg import String

pub = rospy.Publisher('test_topic', String, queue_size=1)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    tN = rospy.Time.now()
    pub.publish(str(tN.secs) + " " + str(tN.nsecs))
    r.sleep()
