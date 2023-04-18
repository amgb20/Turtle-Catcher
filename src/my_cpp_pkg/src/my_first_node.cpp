#include "rclcpp/rclcpp.hpp"

// if i was to write a robot controller node, it would be called MyRobotControllerNode
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0) 
    {
        RCLCPP_INFO(this->get_logger(), "Hello CPP node!"); // get_logger() is a method of the Node class and print "Hello CPP node!" to the console

        timer_ = this->create_wall_timer(             // create a timer
            std::chrono::seconds(1),             // set the timer to 1s
            std::bind(&MyNode::timerCallback, this)); // bind the timer to the timerCallback function and MyNode being a class, we need to pass the this pointer
    }

private:
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello!%d", counter_); // get_logger() is a method of the Node class and print "Hello!" to the console
    }

    rclcpp::TimerBase::SharedPtr timer_; // shared pointer is specific to ros2

    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);               // initialize ROS 2 communication
    auto node = std::make_shared<MyNode>(); // create a shared pointer which has a node
    rclcpp::spin(node);                     // spin the node, expect to receive a shared pointer to a node
    rclcpp::shutdown();                     // shutdown ROS 2 communication
    return 0;
}