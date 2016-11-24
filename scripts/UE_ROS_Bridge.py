import logging
logging.basicConfig(level=logging.DEBUG)
import rospy

import tf
from tf.msg import tfMessage
import geometry_msgs.msg

# imports services to call
# imports messages to publish

import gevent
import gevent.wsgi
import gevent.queue
from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.wsgi import WsgiServerTransport
from tinyrpc.server.gevent import RPCServerGreenlets
from tinyrpc.dispatch import RPCDispatcher

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
    print params
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

# in the main greenlet, run our rpc_server
rpc_server.serve_forever()

