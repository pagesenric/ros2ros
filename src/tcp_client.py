#!/usr/bin/python
import socket
import rospy
from pydoc import locate
from io import BytesIO
import struct

# FUNCTION DEFINITIONS -----------------------------------------------------------------------------------------------------

def callback(req, address, port, srv_type):
    """
    Service callback function to get a service request, send it through TCP to a remote service node and
    retrive it's response.

    Pre:
        - Parameter req must be a ROS service request.
        - Parameters address and port must be a valid IP address combination.
        - Parameter srv_type must be the class of the service message

    Post:
        - The remote service is sent the request req and it's response is returned if the transaction was
          successful.
    """

    rospy.logdebug("[TCP Service Client] Received request. Connecting to " + address + ":" + str(port))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    s.connect((address, port))
    rospy.logdebug("[TCP Service Client] Connected")

    # Serialize request in order to be sent.
    buff = BytesIO()
    req.serialize(buff)

    # Add two null bytes to the end of the message, one for empty messages like Trigger, and one for eof
    buff.write(struct.pack('B', 0))
    buff.write(struct.pack('B', 0))

    s.sendall(buff.getvalue())
    rospy.logdebug("[TCP Service Client] Sent request")
    data = s.recvfrom(4096)
    rospy.logdebug("[TCP Service Client] Received response")
    if not data:
        rospy.logerr("[TCP Service Client] No data received! If empty response, this might not be handled!")
    
    # Process response
    res = srv_type._response_class()
    try:
        res.deserialize(data[0])
    except Exception as e:
        rospy.logerr("[TCP Service Client] Error while deserializing response: " + str(e))
        res = None

    # End connection
    s.shutdown(socket.SHUT_RDWR)
    s.close()
    rospy.logdebug("[TCP Service Client] Closed connection")
    return res

def helper_callback(address, port, srv_type):
    """
    Auxiliary function to send parameters to the TCP Server callback.

    Pre:
        - Parameters address and port must be a valid IP address combination.
        - Parameter srv_type must be the class of the service message
    Post:
        - Returns a lambda function to be used as a Server callback to proxy a remote service and get
          it's response.
    """
    return lambda c : callback(c, address, port, srv_type)

# MAIN CODE ----------------------------------------------------------------------------------------------------------------
rospy.init_node('tcp_service_client')

address = ""

srv_servers = []

# Get service client address

if (rospy.has_param('service_client/address')):
    address = rospy.get_param('service_client/address')
else:
    raise Exception("The parameters 'service_client/address' and 'service_client/port' are not set")

# We go through each service to be proxyed
if (rospy.has_param('service_client/services')):
    services = rospy.get_param('service_client/services')
    #check if every element has a key 'service','type' and 'port'
    for service in services:
        rospy.loginfo("[TCP Service Client] Initializing service " + service)
        if not (rospy.has_param('service_client/services/'+ service + '/service') and rospy.has_param('service_client/services/'+ service + '/type') and rospy.has_param('service_client/services/'+ service + '/port')):
            raise Exception("Every element of the 'services' parameter must have a 'service' and a 'type' key")
        topic = rospy.get_param('service_client/services/'+ service + '/service')
        port = rospy.get_param('service_client/services/'+ service + '/port')
        topic_type = rospy.get_param('service_client/services/'+ service + '/type')

        # Initialize service
        rospy.Service(topic, locate(topic_type), helper_callback(address, port, locate(topic_type)))
       
    rospy.loginfo("[TCP Service Client] Initialized " + str(len(services)) + " services")
else:
    raise Exception("The parameter 'services' is not set")

rospy.spin()