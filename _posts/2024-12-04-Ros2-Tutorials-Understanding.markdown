---
layout: default
title:  "ROS2 Tutorial 1 : Understanding"
date:   2024-12-04
categories: Ros2
---


# ROS2 Tutorial 1 : Understanding
_This tutorial helps you understand how ROS2 works, its components, and provides easy-to-follow examples._

## 1. Configuring environment

ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently.

Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.


### Source the setup files 

```shell
# Possible values are : setup.bash, setup.sh, setup.zsh, it's depend on your terminal 
source /opt/ros/humble/setup.bash   # my ros2 version is humble
```

### Check environment variables 

```shell
printenv | grep -i ROS
```
Output should show : 

```shell
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

if you use multi group of ROS2 nodes , you can set ros_domain_id for each group: 

```shell
export ROS_DOMAIN_ID=<your_domain_id>
```

## 2. ROS2 Components

ROS2 uses a real-time pub/sub model, which includes the following components:  
- node
- topic
- service
- parameter
- action

### 2.1 ROS2 nodes

#### Background

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

[![Alt text](/assets/images/nodes_topic_service.gif)]({{ site.baseurl }}/assets/images/nodes_topic_service.gif)

A full robotic system is comprised of many nodes working in concert

#### Command

```shell
ros2 run <package_name> <executable_name>  # launches an executable from a package
```

```shell
ros2 node list   # show the names of all **running** nodes
```

```shell
ros2 node info <node_name>   #  return list of subscribers, publishers, services, actions. 
```

Remapping 

```shell
ros2 run <package_name> <executable_name> --ros-args --remap __node:<new_package_name>
```

### 2.2 ROS topics
#### Background
ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.
A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

[![Alt text](/assets/images/topic_multi_publisher_multi_subscriber.gif)]({{ site.baseurl }}/assets/images/topic_multi_publisher_multi_subscriber.gif)

#### Command

```shell
ros2 topic list     # return list of all the topics currently active in the system
ros2 topic list -t    # return list of all the topics currently active with the topic type appended
```

```shell
ros2 topic echo <topic_name>    # return data being published on a topic
```

```shell
ros2 topic info <topic_name>    #  show type of topic , number of publisher , number of subscriber
```

```shell
ros2 interface show <msg_name>  # show message structure detail
```

```shell
ros2 topic pub <topic_name> <msg_type> '<args>'  #  publish data to a topic directly
# Ex : ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
At times you may want to publish data to your topic only once (rather than continuously). To publish your command just once add the `--once` option.
`--once` is an optional argument meaning “publish one message then exit”.
`-w 2` is an optional argument meaning “wait for two matching subscriptions”. This is needed because we have both turtlesim and the topic echo subscribed.

```shell
ros2 topic hz <topic_name>  # view rate at which data is published
```

```shell
ros2 topic bw <topic_name>  # bandwidth used by a topic
```

```shell
ros2 topic find <topic_type>  # return list of available topics of a given type
```


### 2.3 ROS Services

#### Background

Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

[![Alt text](/assets/images/service_multi_service_client.gif)]({{ site.baseurl }}/assets/images/service_multi_service_client.gif)

#### Command

```shell
ros2 service list   # list of currently active services
```
You will see that almost nodes have the same six services with `parameters` in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of.

```shell
ros2 service type <service_name>  # describe how the request and response data of a service is structured
```