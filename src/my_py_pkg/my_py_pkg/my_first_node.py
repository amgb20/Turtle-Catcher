#!/usr/bin/env python3

# template for writing a ROS2 node in python

import rclpy
from rclpy.node import Node

# the class must be named with what we want to do, so if it is a camera driver, it must be named MyCameraDriver
# the class must inherit from the class Node


class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")  # call the constructor of the class Node
        self.counter_ = 0
        self.get_logger().info("Hello ROS2!!!")  # print a message
        # create a timer that will call the function timer_callback every 0.5 seconds
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        # print a message with the counter value)
        self.get_logger().info("Hello" + str(self.counter_))

# this will always be the same


def main(args=None):
    rclpy.init(args=args)  # initialise the ros2 communication

    # the name of the node is not the same as the name of the file
    # the node is not the file, the node is the object that is created
    node = MyNode()  # create the node constructor
    # keep the node alive, later callbacks will be called from the function spin
    rclpy.spin(node)
    rclpy.shutdown()  # programs exit


if __name__ == "__main__":
    main()
