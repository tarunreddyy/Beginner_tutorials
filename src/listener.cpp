// Copyright 2023 Tarun Trilokesh

/**
 * @file listener.cpp
 * @brief Listener node for ROS2 beginner tutorials.
 * @author Tarun Trilokesh
 * @date 11/07/2023
 * @version 2.0
 */

// Required includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class Listener
 * @brief ROS2 subscriber node for the "chatter" topic.
 * 
 * The Listener class defines a ROS2 node that subscribes to the "chatter" topic
 * and prints the received messages to the console.
 */
class Listener : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Listener node.
     * 
     * This constructor initializes the ROS2 node and creates a subscription
     * to the "chatter" topic.
     */
    Listener() : Node("listener") {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            std::bind(&Listener::messageCallback, this, std::placeholders::_1));
    }

 private:
    /**
     * @brief Callback function for received messages.
     * 
     * This function is called whenever a new message is received on the
     * "chatter" topic. It prints the message content to the console.
     * 
     * @param msg The received message.
     */
    void messageCallback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    ///< Subscriber for the "chatter" topic.
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

/**
 * @brief Main function for the Listener node.
 * 
 * This function initializes the ROS2 system, creates a Listener node,
 * and spins the node to keep it running.
 * 
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Execution status.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Listener>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
