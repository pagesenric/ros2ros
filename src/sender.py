#!/usr/bin/python3

# Creator: Enric Pagès Agustí
# University of Girona

import socket
import rospy
from pydoc import locate
from io import BytesIO
import threading
import time

########################################################################################################################
# FUNCTION DEFINITIONS -------------------------------------------------------------------------------------------------
########################################################################################################################

def callback_udp(data, host, port, socket):
    """
    This function is ment to be called back by a Subscriber to send the information
    it is given to a certain IP address thorugh UDP.

    Pre: 
        - Parameter data length mustn't exceed 65,507 bytes.
        - Parameters host and port need to be a valid IP address:port combination.
        - Parameter socket should use SOCK_DGRAM and not be connected.
    Post: 
        - The data is serialized and sent to de host's port thorugh the specified socket.
        - Returns nothing.
    """
    # Serialize the recived data into a buffer
    buff = BytesIO()
    data.serialize(buff)         
    
    rospy.loginfo("[UDP Sender] Sending data (" +str(len(buff.getvalue())) + ") to " + str(host) + ":" + str(port) + " through UDP")
    # Data is sent
    socket.sendto(buff.getvalue(), (host, port))


def callback_tcp(data, socket):
    """
    This function is ment to be called back by a Subscriber to send the information
    it is given to a certain IP address thorugh TCP.

    Pre: 
        - Parameter socket must be SOCK_STREAM socket and must have an active connection to the reciver.
    Post: 
        - The data is serialized and sent to the socket's reciver through TCP.
        - Returns nothing.
    """
    # Serialize the recived data into a buffer
    buff = BytesIO()
    data.serialize(buff)         
    
    # Data is sent
    rospy.loginfo("[TCP Sender] Sending data (" +str(len(buff.getvalue())) + ") to " + str(socket) + " through TCP")
    socket.sendall(buff.getvalue())


def helper_callback_udp(host, port, socket):
    """
    Auxiliary function to send parameters to the UDP Subscriber callback.

    Pre:
        - Parameters host and port need to be a valid IP address:port combination.
        - Parameter socket should use SOCK_DGRAM and not be connected.
    Post:
        - Returns a lambda function to be used as a Subscriber callback to send the recived data
          through UDP to the specified host:port pair using the specified socket.
    """
    return lambda c : callback_udp(c, host, port, socket)

def helper_callback_tcp(socket):
    """
    Auxiliary function to send parameters to the TCP Subscriber callback.

    Pre:
        - Parameter socket must be SOCK_STREAM socket and must have an active connection to the reciver.
    Post:
        - Returns a lambda function to be used as a Subscriber callback to send the recived data
          through TCP to the socket's end.
    """
    return lambda c : callback_tcp(c, socket)


def thread_publish_udp(topic_name, topic_type, host, port):
    """
    Thread target function to send whatever is heard from the specified topic
    through UDP to a specified address.

    Pre:
        - Parameter topic_name must be a valid topic name. 
        - Parameter topic_type must be the class of the topic's messages. 
        - Parameters host and port need to be a valid IP address:port combination.
    Post:
        - A Subscriber is initialized that listents to the specified topic and sends the
          data it recives to the specified address using UDP.
        - Returns nothing.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.logdebug("[UDP sender] Starting thread for " + topic_name)
    rospy.Subscriber(topic_name, topic_type, helper_callback_udp(host, port, s))

def thread_publish_tcp(topic_name, topic_type, host, port):
    """
    Thread target function to send whatever is heard from the specified topic
    through TCP to a specified address.

    Pre:
        - Parameter topic_name must be a valid topic name. 
        - Parameter topic_type must be the class of the topic's messages.
        - Parameters host and port need to be a valid IP address:port combination.
    Post:
        - A Subscriber is initialized that listents to the specified topic and sends the
          data it recives to the specified address using TCP, if the connection is accepted.
          Otherwise, the connection will be reatemped till success.
        - Returns nothing.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.logdebug("[TCP sender] Starting thread for " + topic_name)
    
    # We try to connect to the reciver
    connected = False
    wait_time = 5
    while not connected:
        try:
            rospy.loginfo("[TCP sender] Connecting to " + str(host) + ':' + str(port))
            s.connect((host, port))
            connected = True
        except Exception as e:
            rospy.logwarn("[TCP sender] Failed to connect to " + str(host) + ':' + str(port) + '. Reattempting in ' + str(wait_time) + ' seconds...')
            time.sleep(wait_time)
            wait_time += 3

    # Once connected, we can initialize the subscriber
    rospy.loginfo("[TCP sender] Connection to " + str(host) + ':' + str(port) + ' successful!')
    rospy.Subscriber(topic_name, topic_type, helper_callback_tcp(s))


########################################################################################################################
# MAIN CODE -----------------------------------------------------------------------------------------------------------
########################################################################################################################

rospy.init_node('ros2ros_sender')

threads = []

# We obtain the target connection's host name
host = ""

if rospy.has_param('sender/address'):
    host = rospy.get_param('sender/host')
else:
    raise Exception("The parameter 'sender/host' is not set")

# We go through each specified topic to publish
if (rospy.has_param('sender/topics')):
    topics = rospy.get_param('sender/topics')
    
    #check if every element has a key 'topic', 'type' and 'port'
    for topic in topics:
        rospy.loginfo("[Sender] Initializing topic " + topic)
        if not (rospy.has_param('sender/topics/'+ topic + '/topic') and rospy.has_param('sender/topics/'+ topic + '/type') and rospy.has_param('sender/topics/'+ topic + '/port')):
            raise Exception("Every element of the 'topics' parameter must have a 'topic' and a 'type' key")
        topic_name = rospy.get_param('sender/topics/'+ topic + '/topic')
        topic_port = rospy.get_param('sender/topics/'+ topic + '/port')
        topic_type = rospy.get_param('sender/topics/'+ topic + '/type')

        # We check if another protocol is required
        # By default we send the data through UDP
        if rospy.has_param('sender/topics/' + topic + '/protocol'):
            protocol = rospy.get_param('sender/topics/'+ topic + '/protocol')
        else:
            protocol = 'udp'

        # Create a new thread for each topic
        if protocol == 'udp' or protocol == 'UDP':
            threads.append(threading.Thread(target=thread_publish_udp, args=(topic_name, locate(topic_type), host, topic_port)))
        elif protocol == 'tcp' or protocol == 'TCP':
            threads.append(threading.Thread(target=thread_publish_tcp, args=(topic_name, locate(topic_type), host, topic_port)))
        else:
            raise Exception("Message Protocol " + protocol + " not supported")

        # start the thread
        threads[-1].daemon = True
        threads[-1].start()                            
    rospy.loginfo("[Sender] Sender initialized")
else:
    rospy.logerr("[Sender] The parameter 'topics' is not set")

rospy.spin()