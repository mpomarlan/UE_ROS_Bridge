#!/usr/bin/env python

import logging
logging.basicConfig(level=logging.DEBUG)
import rospy

import tf
from tf.msg import tfMessage
import geometry_msgs.msg

from UE_ROS_Bridge_ListenerBase import SetupListeners, SetupServiceListeners

# imports services to call
# imports messages to publish

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher
from threading import Lock

dispatcher = RPCDispatcher()
transport = WsgiServerTransport(queue_class=gevent.queue.Queue)

# start wsgi server as a background-greenlet
wsgi_server = gevent.wsgi.WSGIServer(('0.0.0.0', 10090), transport.handle)
gevent.spawn(wsgi_server.serve_forever)

rpc_server = RPCServerGreenlets(
    transport,
    JSONRPCProtocol(),
    dispatcher
)

TFPublisher = rospy.Publisher('tf', tfMessage, queue_size=1)
rospy.init_node('UE_ROS_Bridge')
tfBroadcaster = tf.TransformBroadcaster()

serviceHandlers = {}
messagePages = [{}, {}]
messageSendingPage = [0]
pageMutex = Lock()

SetupListeners(pageMutex, messagePages, messageSendingPage)
SetupServiceListeners(pageMutex, messagePages, messageSendingPage, serviceHandlers)

@dispatcher.public
def ROSPublishTF(frame_id, child_frame_id, x, y, z, qx, qy, qz, qw):
    global tfBroadcaster
    tfBroadcaster.sendTransform((float(x), float(y), float(z)),
                                (float(qx), float(qy), float(qz), float(qw)),
                                rospy.Time.now(),
                                childFrameId,
                                frameId)

@dispatcher.public
def ROSPublishTopics(params):
    global messagePages
    global messageSendingPage
    global pageMutex
    tfMessages = tfMessage()
    seqTf = 0
    for message in params:
        topic = message['topic']
        params = message['params']
        if ((topic == '/tf') or (topic == 'tf')):
            msg = geometry_msgs.msg.TransformStamped()
            msg.header.seq = seqTf
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = params['frame_id']
            msg.child_frame_id = params['child_frame_id']
            msg.transform.translation.x = float(params['x'])
            msg.transform.translation.y = float(params['y'])
            msg.transform.translation.z = float(params['z'])
            msg.transform.rotation.x = float(params['qx'])
            msg.transform.rotation.y = float(params['qy'])
            msg.transform.rotation.z = float(params['qz'])
            msg.transform.rotation.w = float(params['qw'])
            seqTf = seqTf + 1
            tfMessages.transforms.append(msg)
        elif (topic == 'huh'):
            print "Placeholder topic"
        else:
            print "Unrecognized topic " + topic
    TFPublisher.publish(tfMessages)
	
    pageMutex.acquire()
    messagePages[messageSendingPage[0]].clear()
    messageSendingPage[0] = messageSendingPage[0] ^ 1
    pageMutex.release()
    
    return {"messages": messagePages[messageSendingPage[0]]}

@dispatcher.public
def ROSCallService(params):
    service = params['service']
    request = params['request']
    if service in serviceHandlers:
        serviceHandlers[service](request)

# in the main greenlet, run our rpc_server
rpc_server.serve_forever()

