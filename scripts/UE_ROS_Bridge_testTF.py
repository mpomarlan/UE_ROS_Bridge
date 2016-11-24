from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://127.0.0.1:10090/')
)

remote_server = rpc_client.get_proxy()

# call a method called 'reverse_string' with a single string argument
result = remote_server.ROSPublishTF("frameRoot", "frameChild", "0", "1", "2", "-1", "0", "1", "0.5")

