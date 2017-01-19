import time
from threading import Thread





from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://192.168.100.174:10090/')
)

remote_server = rpc_client.get_proxy()


topicName = 'test_topic'
topicName = 'display_command'



k = 0

def queryTopics():
    while True:
        time.sleep(0.05)
        result = remote_server.ROSPublishTopics([])
        print "Server answered:"
        print result
prgThread = Thread(target = queryTopics)
prgThread.setDaemon(True)
prgThread.start()

while True:
    #tN = rospy.Time.now()
    #pub.publish("Display Text: " + str(k) + " :: " + str(tN.secs) + " " + str(tN.nsecs))
    #k = k + 1
    time.sleep(1)



## call a method called 'reverse_string' with a single string argument
#result = remote_server.prac2cram_client(neutralization_tasks)
#
#print "Server answered:" 
#print result

