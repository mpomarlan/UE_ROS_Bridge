import rospy
from std_msgs.msg import String

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('node_name')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    tN = rospy.Time.now()
    pub.publish(string(tN.secs) + " " + string(tN.nsecs))
    r.sleep()