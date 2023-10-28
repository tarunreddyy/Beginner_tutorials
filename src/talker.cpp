// Copyright 2023 Tarun Trilokesh

/**
 * @file talker.cpp
 * @author Tarun Trilokesh
 * @date 10/27/2023
 * @version 1.0
 * 
 * @brief Main entry point for the beginner_tutorials publisher node.
 *
 * This program initializes a ROS 2 node, creates a publisher,
 * and publishes a custom string message to the "chatter" topic.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Main function to run the publisher node.
 * 
 * Initializes the ROS 2 node and publisher. Publishes a custom string message
 * to the "chatter" topic at a rate of 10 Hz.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int - Returns 0 on successful execution.
 */
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("talker");

    // Create a publisher
    auto chatter_pub = node->create_publisher<std_msgs::msg::String>(
    "chatter", 10);

    // Set the loop rate
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        // Create and publish a message
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World!";
        chatter_pub->publish(msg);

        // Spin and sleep
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
