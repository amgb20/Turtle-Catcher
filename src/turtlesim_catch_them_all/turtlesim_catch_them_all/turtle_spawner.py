#!/usr/bin/env python3
from functools import partial
import rclpy
import random
import math
from rclpy.node import Node

from turtlesim.srv import Spawn
from turtlesim_catch_them_all_interfaces.msg import Turtle
from turtlesim_catch_them_all_interfaces.msg import TurtleArray
from turtlesim_catch_them_all_interfaces.srv import CatchTurtle
from turtlesim.srv import Kill


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        # create a parameter for the spawn_frequency and turtle_name_prefix
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")

        # create a spawn_frequency object
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value

        # create a prefix for the turtle name
        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value

        # create a counter for the turtle name
        self.turtle_counter_ = 0

        # create a list of alive turtles
        self.alive_turtles_ = []

        # create a publisher
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)

        # create a timer self.spawn_turtle_timer_
        self.spawn_turtle_timer = self.create_timer(1.0/self.spawn_frequency_, self.spawn_new_turtle)

        # create a service server
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle)

    # create a callback_catch_turtle function where we call the kill service from turtlesim_node
    # when we call from the turtle_controller node, catch_turtle service will call this function
    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

        # create a publish_alive_turtles function. this function will publish the list of alive turtles
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    # create a spawn_new_turtle function
    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix + \
            str(self.turtle_counter_)  # create a turtle name

        # need to choose x, y, theta randomly
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_server(name, x, y, theta)  # call the service client

    # create a service client

    def call_spawn_server(self, turtle_name, x, y, theta):
        # spawn is the service name
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, turtle_name=turtle_name, x=x, y=y, theta=theta))

    # create a callback function for the service client
    def callback_call_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            # response.name != "" means that the turtle was spawned successfully
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " is now alive")
                # add the turtle to the list of alive turtles
                new_turtle = Turtle()  # create a new turtle
                new_turtle.name = response.name  # name of the turtle
                new_turtle.x = x  # x position of the turtle
                new_turtle.y = y
                new_turtle.theta = theta
                # add the turtle to the list of alive turtles
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()  # publish the list of alive turtles

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    # create a service client for the kill service from turtlesim_node which is called by the callback_catch_turtle function
    def call_kill_server(self, turtle_name):
        # spawn is the service name
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill, turtle_name=turtle_name))

    # create a callback function kill service client, so when it killed the turtle, it will remove the turtle from the list of alive turtles
    # when the turtlesim_node has replied we remove the turtle from the list of alive turtles
    def callback_call_kill(self, future, turtle_name):
        try:
            future.result()  # if the service call fails, an exception will be raised
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name: # if the turtle is in the list of alive turtles
                    del self.alive_turtles_[i] # remove the turtle from the list of alive turtles
                    self.publish_alive_turtles() # publish the list of alive turtles
                    break
            self.get_logger().info("Turtle " + turtle_name + " is now dead")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
