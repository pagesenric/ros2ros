#!/usr/bin/python
import socket
import rospy
from pydoc import locate
from io import BytesIO
import threading
import struct

# FUNCTION DEFINITIONS -----------------------------------------------------------------------------------------------------

def thread_server(client, srv_type, socket):
    """
    Target thread function that listens for service requests from the referenced socket, proxies with it 
    a local server and replies with whatever the server returns.

    Pre:
        - Parameter client must be a ServiceProxy object
        - Parameter srv_type must be the class of the local service
        - Parameter socket must be a SOCK_STREAM socket
    Post:
        - A service request is heard from the socket. The request is passed to the service referenced by the 
          parameter client and it's response is sent back to the source.
    """
    rospy.logdebug("[TCP Service Server] Starting thread for " + client.resolved_name)

    while not rospy.is_shutdown():
        rospy.logdebug("[TCP Service Server] Waiting for TCP connection for " + client.resolved_name)
        socket.listen(1)
        conn, addr = socket.accept()
        rospy.logdebug("[TCP Service Server] Accepted a TCP connection request for" + client.resolved_name)

        # Read data from connection
        data = conn.recv(4096)
        
        if not data:
            # End of transmission
            rospy.logwarn("[TCP Service Server] Received FIN during data reception on " + client.resolved_name + " Closing connection")
            conn.close()
            break
        
        rospy.logdebug("[TCP Service Server] Received data on " + client.resolved_name)
        req = srv_type._request_class()
        
        # We deserialize the recived data
        try:
            req.deserialize(data[:-1])
        except Exception as e:
            rospy.logerr("[TCP Service Server] Error while deserializing request: " + str(e))
            conn.close()
            break
        
        # The translated request is sent to the local service
        res = client.call(req)

        # Serialize the response
        buff = BytesIO()
        res.serialize(buff)
        buff.write(struct.pack('B', 0)) # add a null byte to the end of the message
        
        # Send the response back to the source
        conn.sendall(buff.getvalue())
        rospy.logdebug("[TCP Service Server] Sent response on " + client.resolved_name)
        #Listen for a FIN for at least 5 seconds
        conn.settimeout(5)
        try:
            data = conn.recv(4096)
            if not data:
                rospy.logdebug("[TCP Service Server] Received FIN on " + client.resolved_name)
                conn.close()
            else:
                rospy.logwarn("[TCP Service Server] Expecting FIN but data received on " + client.resolved_name)
                rospy.logwarn("[TCP Service Server] Data: " + str(data))
                conn.close()

        except socket.timeout:
            rospy.logwarn("[TCP Service Server] No FIN received after 5 secs on " + client.resolved_name)
            conn.close()

# MAIN CODE --------------------------------------------------------------------------------------------------------------- 
rospy.init_node('tcp_service_server')

address = ""

# Get address of the local machine
if (rospy.has_param('service_server/address')):
    address = rospy.get_param('service_server/address')
else:
    raise Exception("The parameters 'server/address' and 'server/port' are not set")

# We go through each specified service
if (rospy.has_param('service_server/services')):
    services = rospy.get_param('service_server/services')
    # Check if every element has a key 'service', 'type' and 'port'
    for service in services:
        rospy.loginfo("[TCP Service Server] Initializing service " + service)
        if not (rospy.has_param('service_server/services/'+ service + '/service') and rospy.has_param('service_server/services/'+ service + '/type') and rospy.has_param('service_server/services/'+ service + '/port')):
            raise Exception("Every element of the 'services' parameter must have a 'service' and a 'type' key")
        topic = rospy.get_param('service_server/services/'+ service + '/service')
        port = rospy.get_param('service_server/services/'+ service + '/port')
        topic_type = rospy.get_param('service_server/services/'+ service + '/type')
        
        # Create the TCP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Wait until the port is available
        retry_count = 0
        while True:
            try:
                s.bind((address, port))
                break
            except:
                if retry_count < 10:
                    rospy.loginfo("[TCP Service Server] Port " + str(port) + " is not available, waiting 2 second")
                else: 
                    rospy.logwarn("[TCP Service Server] Port " + str(port) + " is not available, waiting 2 second")
                rospy.sleep(2)
        
        # Create the proxy object of the service
        prox = rospy.ServiceProxy(topic, locate(topic_type))
        
        # Create a thread for the socket
        th = threading.Thread(target=thread_server, args=(prox, locate(topic_type), s))
        
        # Start the thread
        th.daemon = True
        th.start()
    rospy.loginfo("[TCP Service Server] Initialized " + str(len(services)) + " services")
else:
    raise Exception("The parameter 'services' is not set")

rospy.spin()