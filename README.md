# ROS 2 Custom Message Implementation Tutorial

## **Introduction**
This document provides a comprehensive guide to creating and utilizing custom message types within the ROS 2 framework. The tutorial is divided into sections covering package creation, message definition, and the implementation of publisher and subscriber nodes that use the custom message type.

## Section 0: Package and Custom Message Creation
The initial step involves the creation of a ROS 2 workspace and a source directory. Navigate to your intended workspace location to begin.

```bash
cd ~/[your_workspace_name]/src
```

A dedicated package should be created to manage the custom message definitions. This promotes modularity and reusability. Execute the following command to create a new package.

```bash
ros2 pkg create --build-type ament_cmake msg_interface_example
```

> ⚠️ **Note:** the specified build type is ```ament_cmake```, and the **package** is named ```msg_interface_example```.

Proceed into the newly created package directory and create a subdirectory named ```msg```. This directory is the standard location for message definition files (```.msg```).


```bash
cd msg_interface_example
mkdir msg
```

Within the ``msg``` directory, create a new message definition file. For this tutorial, the file will be named ```TwoTextMessage.msg```. Message file names should adhere to the CamelCase convention.

The resulting directory structure should be as follows:
```
.
├── CMakeLists.txt
├── include
│   └── msg_interface_example
├── msg
│   └── TwoTextMessage.msg
├── package.xml
└── src
```

To enable the build system to recognize and generate the custom message, modifications to ```CMakeLists.txt``` and ```package.xml``` are required.

---

## Section 1: ```CMakeLists.txt``` Configuration

Modify the ```CMakeLists.txt``` file to include the necessary dependencies and message generation directives.

```cmake
cmake_minimum_required(VERSION 3.8)
project(msg_interface_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and load build settings from external packages
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TwoTextMessage.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```
This configuration finds required packages (```ament_cmake```, ```builtin_interfaces```, ```rosidl_default_generators```), specifies the message file(s), and invokes ```rosidl_generate_interfaces``` to handle the code generation for the specified message types.

---

## Section 2: ```package.xml``` Configuration

Next, update the ```package.xml``` file to declare the build and execution dependencies.

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>msg_interface_example</name>
  <version>0.0.0</version>
  <description>Package for custom message interfaces</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

After configuring both files, return to the root of the workspace and compile the packages using **```colcon```**.

```bash
colcon build
```

---

## Section 3: Publisher Node Implementation

Create a new package to house the nodes that will utilize the custom message.

```bash
ros2 pkg create using_my_custom_interface_package --build-type ament_python --dependencies rclpy std_msgs msg_interface_example
```

This command generates a Python-based ROS 2 package with a dependency on the previously created ```msg_interface_example``` package.

Within the source directory of this new package, create a Python script for the publisher node. The script must import the custom message type.

```python
from msg_interface_example.msg import TwoTextMessage
```

The publisher node will instantiate the message, populate its fields, and publish it to a topic.

```python
# Excerpt from the publisher's callback or loop
def publish_msg(self):
    msg = TwoTextMessage()
    msg.stamp = self.get_clock().now().to_msg()
    msg.text_a = 'text_a: {0}'.format(self.count)
    msg.text_b = 'text_b: {0}'.format(self.count * 2)
    
    self.publisher.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.text_a)
    self.get_logger().info('Publishing: "%s"' % msg.text_b)
    self.count += 1
```

---

## Section 4: Subscriber Node Implementation

Similarly, the subscriber node must import the custom message type.

```python
import rclpy
from rclpy.node import Node
from msg_interface_example.msg import TwoTextMessage
```

A callback function is defined to process incoming messages from the subscribed topic.

```python
# Excerpt from the subscriber node
def subscribe_topic_message(self, msg):
    self.get_logger().info('Received: "%s"' % msg.text_a)
    self.get_logger().info('Received: "%s"' % msg.text_b)
```

---

## Section 5: Entry Point Configuration

Finally, configure the ```setup.py``` file in the ```using_my_custom_interface_package``` to define the entry points for the publisher and subscriber nodes, making them executable through ```ros2 run```.

---

# Assignment 2: Practical Application

## 1. Problem Description

**1. Message Creation:** Extend the `msg_interface_example` package by adding a new message definition named `TwoNumSum.msg` with the following fields:

- **num_a** → `int8`
- **num_b** → `int8`
  
**2. Publisher Node:** Within the ```using_my_custom_interface_package```, create a publisher node (```sum_publisher.py```). 
This node should:

- Assign random integer values to the ```num_a``` and ```num_b``` fields.
- Use the node name ```<your_name>_publisher```.
- Publish to the topic named ```numbers```.

**3. Subscriber Node:**Create a subscriber node (```sum_subscriber.py```) that:

- Subscribes to the ```numbers``` topic.
- Logs the received values of ```num_a```, ```num_b```, and their calculated sum
- Use the node name ```<your_name>_subscriber```.

---

## 2. Result

The final results are as shown below, and the files used for the assignment have been uploaded to this repository.

## Figure1. Node Graph(RQT_GRAPH)

<img width="973" height="72" alt="rosgraph" src="https://github.com/user-attachments/assets/dd89f85a-59ea-4a8a-bfe0-0fa1f9ff61f7" />

## Figure2. Output(Terminal)

<img width="1301" height="787" alt="Screenshot from 2025-09-18 19-11-16" src="https://github.com/user-attachments/assets/23a2ce0a-88be-470b-8816-53ee796e34b6" />
