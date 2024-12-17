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

Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

#### Command

```shell
ros2 service list   # list of currently active services
ros2 service list -t   # include type of services
```
You will see that almost nodes have the same six services with `parameters` in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of.

```shell
ros2 service type <service_name>  # describe how the request and response data of a service is structured
```

Each service have only one type. But one type can belong to multi service.
Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.

```shell
ros2 service find <type_name>   # return list service of a specific type
```

```shell
ros2 interface show <type_name>  # return structure of service type
```

SERVICE CALL 

`arguments` part is optional

```shell
ros2 service call <service_name> <service_type> <arguments>
# Ex : ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

### 2.4 ROS2 Parameters

#### Background
A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.

#### Command

```shell
ros2 param list   # return parameters belonging to nodes
```

```shell
ros2 param get <node_name> <parameter_name>   # return current value of a parameter
```

```shell
ros2 param set <node_name> <parameter_name> <value>  # set value for a parameter, only apply for current session, not permanently
```

```shell
ros2 param dump <node_name> # show standard output (stdout) of parameters 
# can export to file : ros2 param dump <node_name> > <file_name>
```

```shell
ros2 param load <node_name> <parameter_file>
```

```shell
ros2 run <package_name> <executable_name> --ros-args --params-file <parameter_file>   # load parameter file on node startup
```

### 2.5 ROS2 Actions

#### Background

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.
[![Alt text](/assets/images/action_single_action_client.gif)]({{ site.baseurl }}/assets/images/action_single_action_client.gif)

When you launch the `/teleop_turtle` node, you will see the following message in your terminal:
```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```
Pay attention to the terminal where the `/turtlesim` node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of `/turtlesim` node.
The **F** key will cancel a goal mid-execution.

Try hitting **D** key, then immeditly hit **G**, you will see the message : _[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal_
This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.

#### Command