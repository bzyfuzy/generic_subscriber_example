# ROS 2 Generic Subscriber Example

[![ROS 2](https://img.shields.io/badge/ROS-2-Jade.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

A ROS 2 package demonstrating a subscriber that handles string and integer messages on separate topics, with corresponding publishers.

## Overview

This package provides an example implementation of a ROS 2 (Jazzy) node that subscribes to two distinct topics:
- `/test_topic_string` for `std_msgs/String` messages
- `/test_topic_int` for `std_msgs/Int32` messages

It includes two publisher nodes:
- `string_publisher`: Publishes incrementing string messages (e.g., "Hello ROS 2: 0")
- `int_publisher`: Publishes incrementing integer messages (e.g., 0, 1, 2, ...)

The subscriber logs the message type and content for each received message, showcasing how to handle multiple message types in a single node.

## Prerequisites

- ROS 2 Jazzy installed
- Python 3.12+
- `colcon` build tool

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros_project/src
   git clone https://github.com/yourusername/ros2_generic_subscriber_example.git
   ```

2. Build the package:
   ```bash
   cd ~/ros_project
   colcon build --packages-select generic_subscriber_example
   ```
   
3. Source the workspace:
   ```bash
   source ~/ros_project/install/setup.bash
   ```

## Usage

1. Run the subscriber in one terminal:

   ```bash
   ros2 run generic_subscriber_example generic_subscriber
   ```
3.  Run a publisher in another terminal (choose one or both):
  - String publisher:

    ```bash
    ros2 run generic_subscriber_example string_publisher
    ```
  - Integer publisher:

    ```bash
    ros2 run generic_subscriber_example int_publisher
    ```
3. Observe the subscriber output, which will log message types and contents:
   
   ```bash
   [INFO] [generic_subscriber]: Received message of type: String
   [INFO] [generic_subscriber]: Message content: Hello ROS 2: 0
   [INFO] [generic_subscriber]: Received message of type: Int32
   [INFO] [generic_subscriber]: Message content: 0
   ```
## Testing
Run the included Flake8 style test:

```bash
cd ~/ros_project
colcon test --packages-select generic_subscriber_example
colcon test-result --all --verbose
```

## Package Structure

```
generic_subscriber_example/
├── generic_subscriber_example/
│   ├── __init__.py
│   ├── generic_subscriber.py
│   ├── int_publisher.py
│   └── string_publisher.py
├── package.xml
├── resource/
│   └── generic_subscriber_example
├── setup.cfg
├── setup.py
├── test/
    └── test_flake8.py
```

## ROS Graph

```
+--------------------+       +---------------------+       +--------------------+
| /string_publisher  | ----> | /test_topic_string  | ----> | /generic_subscriber|
| (publishes String) |       | (std_msgs/String)   |       | (subscribes)       |
+--------------------+       +---------------------+       +--------------------+
                                                                   |
+--------------------+       +---------------------+               |
| /int_publisher     | ----> | /test_topic_int     | --------------+
| (publishes Int32)  |       | (std_msgs/Int32)    |
+--------------------+       +---------------------+
```

## License
Apache License 2.0 - See LICENSE for details.

## Contributing
Pull requests welcome! Please follow ROS 2 development guidelines.

---

**Maintainer**: BzY*FuZy <bzy.fuzy@gmail.com>  
**ROS 2 Documentation**: [Actions Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)

