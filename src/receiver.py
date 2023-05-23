#!/usr/bin/python
import socket
import rospy
from pydoc import locate
import threading
import cv2
from cv_bridge import CvBridge
########################################################################################################################
# CONSTANTS ----------------------------------------------------------------------------------------------------------
########################################################################################################################

BUFFER_SIZE = 65536

########################################################################################################################
# FUNCTION DEFINITIONS -------------------------------------------------------------------------------------------------
########################################################################################################################

def deserialize_and_publish(data, publisher, msg_type, socket):
    """
    Function to deserialize data and publish it with the referenced publisher.
    Pre:
        - Parameter data must be seralizde data.
        - Parameter publisher must be a ROS publisher that publishes messages of the class msg_type.
        - Parameter msg_type must be a valid message class.
        - Parameter socket should be the socket where the data was retrived from.
    Post:
        - The data is deserialized and published through the publisher, if possible.
    """
    msg = msg_type()
    try:
        msg.deserialize(data)
        publisher.publish(msg)    
    except:
        rospy.logwarn("Couldn't deserialize the data recived from " + str(socket.getsockname()))

def thread_republish_udp(address, topic_port, publisher, msg_type):
    """
    Thread target function to deserialize recived UDP packets and publish the data to the specified topic.
    Pre:
        - Parameter address must be the receiver's address.
        - Parameter publisher must be a ROS publisher that publishes messages of the class 
          specified by the parameter msg_type.
    Post:
        - A thread is initalized that listens to the specified port for UDP messages. When data is recived,
        it is deserialized with the class of the param msg_type and it is published by the referenced publisher.
    """
    
    # UDP socket that we bind to the address and port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((address, topic_port))
    
    rospy.loginfo("Starting UDP thread for topic " + publisher.name + ", msg type: " + str(msg_type) + ", port: " + str(topic_port))
    
    while True:
        # Wait for data
        data, addr = s.recvfrom(BUFFER_SIZE)
        rospy.loginfo("Recived data (" +str(len(data)) + ") from " + str(s.getsockname()) + " through UDP")

        # Process data
        deserialize_and_publish(data, pub, msg_type, s)

def thread_republish_tcp(address, topic_port, publisher, msg_type):
    """
    Thread target function to accept a TCP connection packets and publish the data recived to the specified topic.
    Pre:
        - Parameter address must be the receiver's address.
        - Parameter publisher must be a ROS publisher that publishes messages of the class 
          specified by the parameter msg_type.
    Post:
        - A thread is initalized that listens to the specified port for a TCP connection. Once stablished,
        waits for data. When recived, it is deserialized with the class of the param msg_type and it is published 
        by the referenced publisher.
    """
    # TCP socket that we bind to the address and port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((address, topic_port))

    rospy.loginfo("Starting TCP thread for topic " + publisher.name + ", msg type: " + str(msg_type) + ", port: " + str(topic_port))

    # Listen for incoming conneciton
    s.listen(1)
    conn, addr = s.accept()

    rospy.loginfo("Connection for topic " + publisher.name + " from " + str(addr) + " stablished!")

    while True:
        # Wait for data
        data = conn.recv(BUFFER_SIZE)
        rospy.loginfo("Recived data (" +str(len(data)) + ") from " + str(s.getsockname()) + " through TCP")
        
        # Process data
        deserialize_and_publish(data, pub, msg_type, s)

def thread_republish_rtsp(address, publisher, encoding = "bgr8"):
    """
    Thread target function to read an image RTSP stream and publish it as a ROS topic.

    Pre:
        - Parameter address must be a valid video-only RTSP stream url.
        - Parameter publisher must be a publisher that publishes sensor_msgs/Image type messages.
        - Parameter encoding must be the encodign used by the RTSP stream.
    Post:
        - The frames recived from the RTSP stream are published by the referenced publisher.
    """
    # Open video capture from the address
    cap = cv2.VideoCapture(address)

    while(cap.isOpened()):
        # Read frame of the video
        ret, frame = cap.read()
        
        # Convert frame to sensor_msgs/Image message
        bridge = CvBridge()
        frame = bridge.cv2_to_imgmsg(frame, encoding)

        # Publish the frame
        publisher.publish(frame)

########################################################################################################################
# MAIN CODE -----------------------------------------------------------------------------------------------------------
########################################################################################################################

rospy.init_node('ros2ros_receiver')

address = ""
threads = []

# We obtain the local address that will recive the connections
if (rospy.has_param('receiver/address')):
    address = rospy.get_param('receiver/address')
else:
    raise Exception("The parameter 'receiver/address' is not set")

# We go through each specified topic to listen.
if (rospy.has_param('receiver/topics')):
    topics = rospy.get_param('receiver/topics')

    # Check if every element has a key 'topic', 'type' and 'port'
    for topic in topics:
        rospy.loginfo("Initializing topic " + topic)
        if not (rospy.has_param('receiver/topics/'+ topic + '/topic') and rospy.has_param('receiver/topics/'+ topic + '/type') and rospy.has_param('receiver/topics/'+ topic + '/port')):
            raise Exception("Every element of the 'topics' parameter must have a 'topic' and a 'type' key")
        topic_name = rospy.get_param('receiver/topics/'+ topic + '/topic')
        topic_port = rospy.get_param('receiver/topics/'+ topic + '/port')
        topic_type = rospy.get_param('receiver/topics/'+ topic + '/type')
        
        # We check if another protocol is required
        # By default we send the data through UDP
        if rospy.has_param('receiver/topics/' + topic + '/protocol'):
            protocol = rospy.get_param('receiver/topics/'+ topic + '/protocol')
        else:
            protocol = 'udp'
              
        # create a publisher for the topic
        pub = rospy.Publisher(topic_name, locate(topic_type), queue_size=10)

        # Create a thread for the topic with the specified protocol
        if protocol == 'udp' or protocol == 'UDP':
            threads.append(threading.Thread(target=thread_republish_udp, args=(address, topic_port, pub, locate(topic_type))))
        elif protocol == 'rtsp' or protocol == 'RTSP':
            if rospy.has_param('receiver/topics/' + topic + '/rtsp_url'):
                rtsp_url = rospy.get_param('receiver/topics/' + topic + '/rtsp_url')
                threads.append(threading.Thread(target=thread_republish_rtsp, args=(rtsp_url, pub, locate(topic_type))))
            else:
                raise Exception("RTSP stream has no URL")
        elif protocol == 'tcp' or protocol == 'TCP':
            threads.append(threading.Thread(target=thread_republish_tcp, args=(address, topic_port,pub, locate(topic_type))))
        else:
            raise Exception("Message Protocol " + protocol + " not supported")
        # start the thread
        threads[-1].daemon = True
        threads[-1].start()

    rospy.loginfo("Initialized " + str(len(topics)) + " topics")
else:
    raise Exception("The parameter 'topics' is not set")

rospy.spin()