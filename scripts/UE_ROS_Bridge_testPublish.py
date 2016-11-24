from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:10090/')
)

remote_server = rpc_client.get_proxy()

# call a method called 'reverse_string' with a single string argument
msgs = [{"topic": "tA", "params": {}}, {"topic": "huh", "params": {}},
        {"topic": "/tf", "params": {"frame_id": "World", "child_frame_id": "Robot", "x": "10.0", "y": "2", "z": "3", "qx": "0.0", "qy": "0.7", "qz": "-0.7", "qw": "0.01"}},
        {"topic": "/tf", "params": {"frame_id": "World", "child_frame_id": "Human", "x": "5.0", "y": "7", "z": "3", "qx": "0.0", "qy": "0.7", "qz": "-0.7", "qw": "0.01"}}]

result = remote_server.ROSPublishTopics(msgs)

