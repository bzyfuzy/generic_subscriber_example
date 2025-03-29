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

"""ROS 2 node that subscribes to string and integer messages on separate topics."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int32, String


class GenericSubscriber(Node):
    """A ROS 2 node that subscribes to string and integer messages on separate topics."""

    def __init__(self):
        """Initialize the GenericSubscriber node with subscriptions to two topics."""
        super().__init__('generic_subscriber')
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Create subscriptions with different topic names
        self.string_sub = self.create_subscription(
            String,
            '/test_topic_string',
            lambda msg: self.callback(msg, 'String'),
            qos
        )
        self.int_sub = self.create_subscription(
            Int32,
            '/test_topic_int',
            lambda msg: self.callback(msg, 'Int32'),
            qos
        )
        self.get_logger().info('Generic Subscriber started, waiting for messages...')

    def callback(self, msg, msg_type):
        """Handle incoming messages and log their type and content.

        Args:
            msg: The received message (String or Int32)
            msg_type (str): The type of the message ('String' or 'Int32')
        """
        self.get_logger().info(f'Received message of type: {msg_type}')
        self.get_logger().info(f'Message content: {msg.data}')


def main(args=None):
    """Run the generic subscriber node."""
    rclpy.init(args=args)
    node = GenericSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
