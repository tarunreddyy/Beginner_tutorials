// Copyright 2023 Tarun Trilokesh

/**
 * @file listener.cpp
 * @author Tarun Trilokesh
 * @date 10/27/2023
 * @version 1.0
 *
 * @brief Main entry point for the beginner_tutorials subscriber node.
 *
 * This program initializes a ROS 2 node, creates a subscriber,
 * and subscribes to the "chatter" topic to receive string messages.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Callback function to handle received messages.
 * 
 * This function is called whenever a new message is received on
 * the "chatter" topic. It simply prints the message to the console.
 * 
 * @param msg The received message.
 */
void chatterCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("listener"), "I heard: '%s'", msg->data.c_str());
}

/**
 * @brief Main function to run the subscriber node.
 * 
 * Initializes the ROS 2 node and subscriber. Continuously listens for
 * messages on the "chatter" topic and calls the callback function to
 * handle them.
 * 
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int - Returns 0 on successful execution.
 */
int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a node
    auto node = rclcpp::Node::make_shared("listener");

    // Create a subscriber
    auto chatter_sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, chatterCallback);

    // Spin to handle incoming messages
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}