#!/usr/bin/python3
import socket
import rospy
from pydoc import locate
import threading
import cv2
from sensor_msgs.msg import Image
import os
import time
#from cv_bridge import CvBridge
########################################################################################################################
# CONSTANTS ----------------------------------------------------------------------------------------------------------
########################################################################################################################

BUFFER_SIZE = 655360
WAIT_TIME = 5

########################################################################################################################
# FUNCTION DEFINITIONS -------------------------------------------------------------------------------------------------
########################################################################################################################

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

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
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((address, topic_port))
    
    rospy.loginfo("Starting UDP thread for topic " + publisher.name + ", msg type: " + str(msg_type) + ", port: " + str(topic_port))
    
    while True:
        # Wait for data
        data, addr = s.recvfrom(BUFFER_SIZE)
        rospy.loginfo("[UDP Receiver] Recived data (" +str(len(data)) + ") from " + str(s.getsockname()) + " through UDP")

        # Process data
        msg = msg_type()
        try:
            msg.deserialize(data)
            print(msg)
            publisher.publish(msg)    
        except:
            rospy.logwarn("[UDP Receiver] Couldn't deserialize the data recived from " + str(s.getsockname()))

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
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((address, topic_port))

    while True:
        rospy.loginfo("[TCP Receiver] Starting TCP thread for topic " + publisher.name + ", msg type: " + str(msg_type) + ", port: " + str(topic_port))
        # Listen for incoming conneciton
        s.listen(1)
        conn, addr = s.accept()

        rospy.loginfo("[TCP Receiver] onnection for topic " + publisher.name + " from " + str(addr) + " stablished!")
        data = 1
        while data:
            # Wait for data
            data = conn.recv(BUFFER_SIZE)
            
            if data:
                rospy.loginfo("Recived data (" +str(len(data)) + ") from " + str(s.getsockname()) + " through TCP")
                
                # Process data
                msg = msg_type()
                # try:
                msg.deserialize(data)
                # print(msg)
                publisher.publish(msg)    
                # except:
                #     rospy.logwarn("Couldn't deserialize the data recived from " + str(s.getsockname()))
        
        rospy.loginfo("Connection for topic " + publisher.name + " from " + str(addr) + " lost...")

def thread_republish_rtsp(address, publisher):
    """
    Thread target function to read an image RTSP stream and publish it as a ROS topic.

    Pre:
        - Parameter address must be a valid video-only RTSP stream url.
        - Parameter publisher must be a publisher that publishes sensor_msgs/Image type messages.
    Post:
        - The frames recived from the RTSP stream are published by the referenced publisher.
    """
    # Set FFMPEG capture options
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
    # Open video capture from the address
    connected = False
    wait_time = WAIT_TIME
    while True:
        cap = cv2.VideoCapture(address, cv2.CAP_FFMPEG)
        while(cap.isOpened()):
            if not connected:
                rospy.loginfo("Reading stream from " + address)
                connected = True
            try:
                # Read frame of the video
                ret, frame = cap.read()

                # Convert frame to sensor_msgs/Image message
                #bridge = CvBridge()
                #frame = bridge.cv2_to_imgmsg(frame, encoding)
                frame = cv2_to_imgmsg(frame)

                # Publish the frame
                publisher.publish(frame)
            except:
                cap.release()
        if not connected:
            rospy.logwarn("Couldn't connect to " + address + ". Reattempting in " + str(wait_time) + " seconds")
            time.sleep(wait_time)
            wait_time += 3
        else:
            connected = False
            wait_time = WAIT_TIME
            rospy.logwarn("RTSP stream from url " + address + " closed. Reattempting connection")

    

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
        rospy.loginfo("[Receiver] Initializing topic " + topic)
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
                threads.append(threading.Thread(target=thread_republish_rtsp, args=(rtsp_url, pub)))
            else:
                rospy.logerr("RTSP stream has no URL")
        elif protocol == 'tcp' or protocol == 'TCP':
            threads.append(threading.Thread(target=thread_republish_tcp, args=(address, topic_port,pub, locate(topic_type))))
        else:
            raise Exception("Message Protocol " + protocol + " not supported")
        # start the thread
        threads[-1].daemon = True
        threads[-1].start()

    rospy.loginfo("[Receiver] Initialized " + str(len(topics)) + " topics")
else:
    rospy.logerr("[Receiver] The parameter 'topics' is not set")


rospy.spin()