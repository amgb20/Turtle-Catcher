#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class SmartphoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")

        self.number_ = 0

        #create a subscriber, the subscriber will subscribe to a message of type String on the topic robot_news
        self.subscriber_ = self.create_subscription(
            String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started.")

    def callback_robot_news(self, msg):
        self.get_logger().info(msg.data)

        # create a publisher, the publisher will publish a message of type Int64 on the topic number
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        # create a timer that will call the function publish_number every 0.5 seconds
        self.timer_ = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("Number Publisher has been started")


def main(args=None):
    rclpy.init(args=args)
    node = SmartphoneNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()