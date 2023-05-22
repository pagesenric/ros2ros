# ros2ros
ROS package to comunicate between two separate rosmasters

## Description
This package allows two separate ROS machines with separate rosmasters to lisent and publish to eachother's topics and proxy eachother's services.

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
WIP
