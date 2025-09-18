# ROS Custom Message Tutorial

**Abstract**  
This repository provides a complete, ready-to-publish tutorial for creating and using custom messages in ROS 2. It covers defining custom message types, configuring build files, implementing publisher/subscriber nodes in Python, and an example assignment (TwoNumSum). The document is intended to be dropped directly into a GitHub `README.md`.

---

## Prerequisites

- ROS 2 (Foxy / Galactic / Humble or later)
- `colcon` build tools
- Python 3.8+ (for ament_python packages)
- Basic familiarity with ROS 2 nodes, topics, and packages

---

## Repository structure (expected after following tutorial)

```
your_workspace/
└── src/
    ├── msg_interface_example/          # custom message package (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── TwoTextMessage.msg
    │       └── TwoNumSum.msg
    └── using_my_custom_interface_package/  # example publisher/subscriber (ament_python)
        ├── package.xml
        ├── setup.py
        └── using_my_custom_interface_package/
            ├── __init__.py
            ├── two_text_publisher.py
            ├── two_text_subscriber.py
            ├── sum_publisher.py
            └── sum_subscriber.py
```

---

## 0. Workspace and Package Setup

1. Create a ROS 2 workspace and `src` folder:

```bash
mkdir -p ~/ros_custom_msgs_ws/src
cd ~/ros_custom_msgs_ws/src
```

2. Create the custom message package (CMake / rosidl):

```bash
ros2 pkg create --build-type ament_cmake msg_interface_example
cd msg_interface_example
mkdir msg
```

3. Create message files inside `msg/`:
- `TwoTextMessage.msg`
- `TwoNumSum.msg`

---

## 1. Message definitions

Create `msg/TwoTextMessage.msg` with:

```plaintext
# TwoTextMessage.msg
builtin_interfaces/Time stamp
string text_a
string text_b
```

Create `msg/TwoNumSum.msg` with:

```plaintext
# TwoNumSum.msg
int8 num_a
int8 num_b
```

---

## 2. msg_interface_example — CMakeLists.txt

Edit `msg_interface_example/CMakeLists.txt` to include message generation:

```cmake
cmake_minimum_required(VERSION 3.8)
project(msg_interface_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/TwoTextMessage.msg"
  "msg/TwoNumSum.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

---

## 3. msg_interface_example — package.xml

Edit `msg_interface_example/package.xml` (minimum relevant parts):

```xml
<?xml version="1.0"?>
<package format="3">
  <name>msg_interface_example</name>
  <version>0.0.0</version>
  <description>Custom message definitions for tutorial</description>
  <maintainer email="your.email@example.com">yourname</maintainer>
  <license>Apache-2.0</license>

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

---

## 4. Build the workspace (first build)

Return to workspace root and build the `msg_interface_example` package so other packages can depend on the generated interfaces:

```bash
cd ~/ros_custom_msgs_ws
colcon build --packages-select msg_interface_example
source install/setup.bash
```

---

## 5. Create using package (Python) that uses the custom messages

From `src/`:

```bash
cd ~/ros_custom_msgs_ws/src
ros2 pkg create using_my_custom_interface_package --build-type ament_python --dependencies rclpy msg_interface_example
```

This creates an ament_python package. We'll add four example scripts and `setup.py` entry points.

---

## 6. using_my_custom_interface_package — package.xml

Edit `using_my_custom_interface_package/package.xml` to ensure dependency on `msg_interface_example`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>using_my_custom_interface_package</name>
  <version>0.0.0</version>
  <description>Example nodes using custom messages</description>
  <maintainer email="your.email@example.com">yourname</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>msg_interface_example</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 7. using_my_custom_interface_package — setup.py

Place this `setup.py` at the package root:

```python
from setuptools import setup

package_name = 'using_my_custom_interface_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='your.email@example.com',
    description='Example publisher/subscriber using custom ROS2 messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'two_text_publisher = using_my_custom_interface_package.two_text_publisher:main',
            'two_text_subscriber = using_my_custom_interface_package.two_text_subscriber:main',
            'sum_publisher = using_my_custom_interface_package.sum_publisher:main',
            'sum_subscriber = using_my_custom_interface_package.sum_subscriber:main',
        ],
    },
)
```

---

## 8. Node implementations (Python)

Create the module folder `using_my_custom_interface_package/` and add `__init__.py` (can be empty). Then create these four scripts:

### two_text_publisher.py

```python
# using_my_custom_interface_package/two_text_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoTextMessage

class TwoTextPublisher(Node):
    def __init__(self):
        super().__init__('yourname_two_text_publisher')  # replace 'yourname'
        self.publisher = self.create_publisher(TwoTextMessage, 'two_text', QoSProfile(depth=10))
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.count = 0

    def publish_msg(self):
        msg = TwoTextMessage()
        msg.stamp = self.get_clock().now().to_msg()
        msg.text_a = f'text_a: {self.count}'
        msg.text_b = f'text_b: {self.count * 2}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.text_a} | {msg.text_b}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TwoTextPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### two_text_subscriber.py

```python
# using_my_custom_interface_package/two_text_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.msg import TwoTextMessage

class TwoTextSubscriber(Node):
    def __init__(self):
        super().__init__('yourname_two_text_subscriber')  # replace 'yourname'
        self.sub = self.create_subscription(TwoTextMessage, 'two_text', self.callback, QoSProfile(depth=10))

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.text_a} | {msg.text_b}')

def main(args=None):
    rclpy.init(args=args)
    node = TwoTextSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### sum_publisher.py

```python
# using_my_custom_interface_package/sum_publisher.py
import rclpy
from rclpy.node import Node
from msg_interface_example.msg import TwoNumSum
import random

class SumPublisher(Node):
    def __init__(self):
        super().__init__('yourname_publisher')  # replace 'yourname'
        self.pub = self.create_publisher(TwoNumSum, 'numbers', 10)
        self.timer = self.create_timer(1.0, self.publish_values)

    def publish_values(self):
        a = random.randint(-128, 127)  # int8 range
        b = random.randint(-128, 127)
        msg = TwoNumSum()
        msg.num_a = int(a)
        msg.num_b = int(b)
        self.pub.publish(msg)
        self.get_logger().info(f'Published num_a={a}, num_b={b}')

def main(args=None):
    rclpy.init(args=args)
    node = SumPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### sum_subscriber.py

```python
# using_my_custom_interface_package/sum_subscriber.py
import rclpy
from rclpy.node import Node
from msg_interface_example.msg import TwoNumSum

class SumSubscriber(Node):
    def __init__(self):
        super().__init__('yourname_subscriber')  # replace 'yourname'
        self.sub = self.create_subscription(TwoNumSum, 'numbers', self.callback, 10)

    def callback(self, msg):
        total = int(msg.num_a) + int(msg.num_b)
        self.get_logger().info(f'Received: {msg.num_a} + {msg.num_b} = {total}')

def main(args=None):
    rclpy.init(args=args)
    node = SumSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

> Replace `'yourname'` in node names with your actual name or desired identifier to match assignment requirements.

---

## 9. Build complete workspace

From workspace root:

```bash
cd ~/ros_custom_msgs_ws
colcon build --symlink-install
source install/setup.bash
```

If you add or modify message packages, rebuild `msg_interface_example` first then rebuild dependent packages.

---

## 10. Run nodes (examples)

Open separate terminals (source `install/setup.bash` in each):

Run the two-text demo:

```bash
ros2 run using_my_custom_interface_package two_text_publisher
ros2 run using_my_custom_interface_package two_text_subscriber
```

Run the assignment demo (random numbers + sum):

```bash
ros2 run using_my_custom_interface_package sum_publisher
ros2 run using_my_custom_interface_package sum_subscriber
```

Inspect topics:

```bash
ros2 topic list
ros2 topic echo /numbers
ros2 topic echo /two_text
```

Visualize node/topic graph:

```bash
rqt_graph
```

---

## 11. Expected outputs (examples)

**Publisher terminal (sum_publisher)**

```
[INFO] [yourname_publisher]: Published num_a=12, num_b=-7
[INFO] [yourname_publisher]: Published num_a=-120, num_b=50
...
```

**Subscriber terminal (sum_subscriber)**

```
[INFO] [yourname_subscriber]: Received: 12 + -7 = 5
[INFO] [yourname_subscriber]: Received: -120 + 50 = -70
...
```

**TwoText messages**

```
[INFO] [yourname_two_text_subscriber]: Received: text_a: 0 | text_b: 0
[INFO] [yourname_two_text_subscriber]: Received: text_a: 1 | text_b: 2
...
```

---

## 12. Assignment Instructions (concise)

- Add `TwoNumSum.msg` to `msg_interface_example/msg/`.
- Implement `sum_publisher.py` and `sum_subscriber.py` as shown.
- Ensure package `using_my_custom_interface_package` depends on `msg_interface_example`.
- Use node names: `<your_name>_publisher` and `<your_name>_subscriber`.
- Topic name: `numbers`.

---

## 13. Example figures

**Figure 1 — ROS graph (rqt_graph)**  
![Rqt_graph](https://github.com/user-attachments/assets/474c3ef1-5ae5-45ba-b8d5-7e5934f9d536)

**Figure 2 — Terminal output**  
![Terminal Result](https://github.com/user-attachments/assets/92562cdf-da7b-42e9-957d-adee67eb7bfc)

(Replace images or paths with screenshots from your runs if required.)

---

## 14. Troubleshooting

- If Python entry points are not found after `colcon build`, ensure `setup.py` and package folder names match, and `install/setup.bash` is sourced.
- If custom message types are missing in dependent packages, rebuild `msg_interface_example` first and re-source `install/setup.bash`.
- Use `ros2 interface show msg_interface_example/TwoNumSum` to confirm generated interfaces.
- For type range errors: `int8` values must be between -128 and 127.

---

## 15. References

- ROS 2 documentation — Custom interfaces & messages  
  https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html
- builtin_interfaces/Time reference  
  https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html

---

## 16. License & Maintainer

**License:** Apache-2.0 (or choose desired license)  
**Maintainer:** Jinhan Lee — dlwlsgks3579@gmail.com

---

**Notes:**  
- This README is ready to be used as `README.md`.  
- Replace placeholder names/emails with your real details.  
- If you want, I can generate a compact `README` variation (shorter) or provide ready-to-paste `package.xml`, `CMakeLists.txt`, and Python scripts as separate files — tell me which format you prefer.
