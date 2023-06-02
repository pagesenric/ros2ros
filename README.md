# ros2ros
ROS package to comunicate between two separate rosmasters

## Description
This package allows two separate ROS machines with separate rosmasters to lisent and publish to eachother's topics and proxy eachother's services.

## Dependencies
- ROS
- CV2 (OpenCV)
- gstreamer libs:
```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```

## Usage
For each roscore, use the following command to enable the publisher/listener comunication between rosmasters.
```
roslaunch ros2ros comunicacio_topics.launch
```

To enable the service's comunication, use the following command
```
roslaunch ros2ros comunicacio_services.launch
```

## YAML Setup
In order to work properly, each machine must have the config YAMLs setup to allow comunication from the other machine.
### config_receiver.yaml
This YAML configures which topics are read from external connections.

The local address must be given under the label "address".
Decleare each topic to be recievd under the label "topics". There can be declared as many topics as desired, as long as they are given a unique port. Each topic must provide the topic's name, the kind of messages it uses, and the port it is expected to be read from. It can additionally provide which protocol is used to send the topics information (tcp, udp or rtsp). By default it will be set as UDP. If the protocol set is RTSP, it must be provided the url of the stream. Currenty, only video-based streams are supported.
Everything must be under the label "receiver".

The file should follow this pattern:
```
receiver:
  address: 'xxx.xxx.xxx.xxx' # Local address
  
  topics:
    example_topic:
      topic: "/topic_name" 
      type: "package_family.msg.packageName"
      port: 10006
      protocol: "tcp"
    example_stream:
      topic: "/topic_name" 
      type: "sensor_msgs.msg.Image"
      port: 10010
      protocol: "rtsp"
      rtsp_url: "rtsp://ipaddressofserver:port/mountpoint"
```

Each declared topic should be also delcared with the same parameters in the file config_sender.yaml of the opposite machine, except for RTSP streams. Those must be declared in the file stream_setup.yaml.

### config_sender.yaml
This YAML configures which topics are sent to external connections.

The target connection address must be given under the label "address".
Decleare each topic to be sent under the label "topics". There can be declared as many topics as desired, as long as they are given a unique port. Each topic must provide the topic's name, the kind of messages it uses, and the port it is expected to be read from. It can additionally provide which protocol is used to send the topics information (tcp or udp). By default it will be set as UDP. To send data through RTSP, refer to the stream_setup.yaml file.
Everything must be under the label "receiver".

The file should follow this pattern:
```
receiver:
  address: 'xxx.xxx.xxx.xxx' # Target address
  
  topics:
    example_topic:
      topic: "/topic_name" 
      type: "package_family.msg.packageName"
      port: 10006
      protocol: "tcp"
```

Each declared topic should be also delcared with the same parameters in the file config_receiver.yaml of the opposite machine.

### stream_setup.yaml
This YAML configures which Image based topics are sent to external connections via RTSP.

The port used to send the streams must be given under the label "port".
Decleare each stream to be recievd under the label "streams". There can be declared as many streams as desired, as long as they are given a unique mountpoint. Each stream must provide the source name, the source type, the mountpoint of the stream, the video restrictions, and the target bitrate.

The file should follow this pattern:
```
port: "8554"
streams:
  example_stream:
    type: topic
    source: /videofile/image_raw # Image rostopic. It must be raw image! Don't use transport topics!
    mountpoint: /back 
    caps: video/x-raw,framerate=10/1,width=640,height=480 # Constraints of the video stream
    bitrate: 500 # Target bitrate of the stream
```
Each declared stream should be also delcared with the same parameters in the file config_receiver.yaml of the opposite machine.

### config_tcp_service.yaml
This YAML configures which services are comunicated with external sources.

The local address must be given under the label "address".
Decleare each service under the label "services". There can be declared as many services as desired, as long as they are given a unique port. Each service must provide the service's name, the kind of service it is, and the port it is expected to be read from.
Everything must be under the label "service_server".

The file should follow this pattern:
```
service_server:
  address: 'xxx.xxx.xxx.xxx' # Local address
  
  services:
    example_topic:
      service: "/service_name" 
      type: "service_family.srv.serviceClassName"
      port: 10006
```

Each declared service should be also delcared with the same parameters in the file config_tcp_client.yaml of the opposite machine.

### config_tcp_client.yaml
This YAML configures which external services are able to be proxied.

The target connection address must be given under the label "address".
Decleare each service under the label "services". There can be declared as many services as desired, as long as they are given a unique port. Each service must provide the service's name, the kind of service it is, and the port it is expected to be read from.
Everything must be under the label "service_client".

The file should follow this pattern:
```
service_client:
  address: 'xxx.xxx.xxx.xxx' # Target address
  
  services:
    example_topic:
      service: "/service_name" 
      type: "service_family.srv.serviceClassName"
      port: 10006
```

Each declared service should be also delcared with the same parameters in the file config_tcp_service.yaml of the opposite machine.
