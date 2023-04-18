#!/usr/bin/env python3

# diagram representing the rqt graph of the system
# turtle_spawn.py --> turtle_controller.py --> turtlesim_node.py

from functools import partial
import rclpy
import math
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_catch_them_all_interfaces.msg import Turtle
from turtlesim_catch_them_all_interfaces.msg import TurtleArray
from turtlesim_catch_them_all_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # # create a temporary target x and y
        # self.target_x = 1.0
        # self.target_y = 10.0

        # declare a parameter for the catch_closest_turtle_first
        self.declare_parameter("catch_closest_turtle_first", True)

        # create a turtle_to_catch object
        self.turtle_to_catch_ = None

        # create a pose object
        self.pose = None

        # create a catch_closest_turtle_first
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value

        # create a subscriber
        # /turtle1/pose is the topic name, self.callback_turtle_pose is the callback function, 10 is the queue size
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)

        # create a publisher
        # /turtle1/cmd_vel is the topic name, Twist is the message type, 10 is the queue size
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)

        # create a subscriber
        # /alive_turtles is the topic name, self.callback_alive_turtles is the callback function, 10 is the queue size
        self.alive_turtles_subscriber = self.create_subscription(
            TurtleArray, "/alive_turtles", self.callback_alive_turtles, 10)

        # create control_loop_timer_
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    # create a callback function for the subscriber
    def callback_turtle_pose(self, msg):
        self.pose = msg  # whenever we receive a message, we update the pose object
        self.get_logger().info("Turtle pose: x=%f, y=%f" % (msg.x, msg.y))

    # this callback will receive the list of alive turtles and it is now updated by the turtle_spawner node when a turtle is killed and the array is updated
    def callback_alive_turtles(self, msg):
        # if the list of alive turtles is not empty we need to choose one of them
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    # we calculate the distance between the current position and the target position
                    dist_x = turtle.x - self.pose.x
                    dist_y = turtle.y - self.pose.y
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

                    # we check if the distance is smaller than the previous one
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle  # we update the closest turtle
                        closest_turtle_distance = distance  # we update the closest turtle distance
                    self.turtle_to_catch_ = closest_turtle 
            else:
                # we choose the first one
                self.turtle_to_catch_ = msg.turtles[0]

        # create a control loop function to move the turtle
    def control_loop(self):

        # if the pose object is empty, return
        # we check if there is a turtle to catch and a position to go to.
        # we dont go in the control loop if we dont have a turtle to catch or a position to go to
        if self.pose is None or self.turtle_to_catch_ == None:
            return

        # we need to calculate the distance between the current position and the target position
        # to do this we are using the pythagore theorem
        # the distance is the hypotenuse of the triangle
        # the distance is the square root of the sum of the squares of the other two sides
        # the distance is the square root of (x2 - x1)^2 + (y2 - y1)^2
        # the distance is the square root of (target_x - current_x)^2 + (target_y - current_y)^2
        dist_x = self.turtle_to_catch_.x - self.pose.x
        dist_y = self.turtle_to_catch_.y - self.pose.y
        distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

        # create a Twist message
        msg = Twist()

        # we need to know if the turtle has reached the target position and we set a tolerance of 0.5
        if distance > 0.5:
            # simple proportional controller
            # you check the delta between the current position and the target position
            # we multiply the distance by a gain to make the turtle move faster and behave better
            msg.linear.x = 2*distance

            # orientation that we want the turtle to have by computing it with actan2 (y, x)
            goal_theta = math.atan2(dist_y, dist_x)
            # we calculate the difference between the current orientation and the goal orientation to feed the controller
            diff = goal_theta - self.pose.theta
            # normalise the angle to be between -pi and pi (to avoid the turtle to turn in the wrong direction) and weird behaviours of the turtle going out of the edge of the screen and trigonometry circle
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            # we multiply the difference by a gain to make the turtle turn faster and behave better
            msg.angular.z = 6*diff

        else:
            # target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # we ask the server to catch the turtle and kill it
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            # however, until we receive a new array with new turtles, this turtle is still the turtle to catch.
            # so at 100HZ, it will still call 'call_catch_turtle_server' 100 times per second
            # so once we have found it, we set it to None so that we dont call the server anymore
            # so when we have reach the target, it is useless to call the server anymore
            self.turtle_to_catch_ = None

        # publish the message and we see the turtle moving
        self.cmd_vel_publisher.publish(msg)

        # create a service client with the turtle server
    def call_catch_turtle_server(self, turtle_name):
        # spawn is the service name
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server ...")

        request = CatchTurtle.Request()  # we create a request object
        request.name = turtle_name  # we set the name of the turtle to catch

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    # create a callback function for the service client
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()  # we check the reponse of the service as it is a boolean
            if not response.success:
                # if the response is false, the turtle is not dead
                self.get_logger().info("Turtle " + turtle_name + " is not dead")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
