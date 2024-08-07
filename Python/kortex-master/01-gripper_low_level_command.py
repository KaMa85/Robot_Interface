import sys
import time
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TransportClientTcp import TransportClientTcp
from kortex_api.TransportClientUdp import TransportClientUdp

# Assuming you have a module named utilities for connection arguments
# If not, replace the following line with direct assignment:
# IP_ADDRESS = "192.168.x.x"
# PORT = 10000 for TCP, 10001 for UDP
import utilities

def main():
    args = utilities.parseConnectionArguments()

    # Create the router for TCP communication
    tcp_transport = TransportClientTcp()
    tcp_router = RouterClient(tcp_transport)
    tcp_transport.connect(args.ip_address, args.port)

    # Create session
    session_manager = SessionManager(tcp_router)
    session_info = Session_pb2.CreateSessionInfo()
    session_info.username = args.username
    session_info.password = args.password
    session_info.session_inactivity_timeout = 60000  # milliseconds
    session_info.connection_inactivity_timeout = 2000  # milliseconds
    session_manager.CreateSession(session_info)

    # Create the router for UDP communication
    udp_transport = TransportClientUdp()
    udp_router = RouterClient(udp_transport)
    udp_transport.connect(args.ip_address, args.udp_port)

    # Instantiate the base client and base cyclic client
    base = BaseClient(tcp_router)
    base_cyclic = BaseCyclicClient(udp_router)

    # Your logic to move to home position and set up low-level control

    # Cleanup
    session_manager.CloseSession()
    tcp_transport.disconnect()
    udp_transport.disconnect()

if __name__ == "__main__":
    main()
