#!/usr/bin/env python3
# Copyright 2025 BzY*FuZy <bzy.fuzy@gmail.com>
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 node that publishes string messages."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class StringPublisher(Node):
    """A ROS 2 node that publishes string messages with an incrementing counter."""

    def __init__(self):
        """Initialize the StringPublisher node with a publisher."""
        super().__init__('string_publisher')
        self.publisher = self.create_publisher(
            String,
            '/test_topic_string',
            10
        )
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """Publish a string message with an incrementing counter."""
        msg = String()
        msg.data = f'Hello ROS 2: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing String: "{msg.data}"')
        self.count += 1


def main(args=None):
    """Run the string publisher node."""
    rclpy.init(args=args)
    node = StringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
