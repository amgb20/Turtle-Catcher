from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publisher_Node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[
            remap_number_topic
        ],
        parameters=[
            {"number_to_publish": 2},
            {"frequency_to_publish": 5.0}
        ]
    )

    number_counter_node = Node(
        package="my_py_pkg",
        executable="number_counter",
        name="my_number_counter",
        remappings=[
            remap_number_topic,
            ("number_count", "my_number_count")]
    )

    # add the node to the launch description
    ld.add_action(number_publisher_Node)
    ld.add_action(number_counter_node)

    return ld
