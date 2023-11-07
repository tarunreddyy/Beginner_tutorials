// Copyright 2023 Tarun Trilokesh

/**
 * @file talker.cpp
 * @author Tarun Trilokesh
 * @date 10/29/2023
 * @version 2.0
 * 
 * @brief Main entry point for the beginner_tutorials publisher node.
 *
 * This program initializes a ROS 2 node, creates a publisher,
 * and publishes a custom string message to the "chatter" topic.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

/**
 * @brief A Talker class for ROS2
 * @details This class represents a ROS2 node that publishes messages and provides a service
 * to change the base string.
 */
class Talker : public rclcpp::Node {
 public:
    /**
     * @brief Default constructor for Talker
     * @details Initializes the publisher, service, and sets the base_string_ to "Hello World!"
     */
    Talker() : Node("talker"), base_string_("Hello World!") {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
                    "chatter", 10);
        service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
                    "change_string",
                    std::bind(&Talker::handleChangeStringRequest, this,
                    std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO_STREAM(this->get_logger(),
                    "Talker node initialized with base string: "
                    << base_string_);
    }

    /**
     * @brief Publish the base string as a message
     * @details This function publishes the base_string_ as a message to the "chatter" topic.
     */
    void publishMessage() {
        if (base_string_.empty()) {
            RCLCPP_FATAL(this->get_logger(),
            "Base string is empty. Cannot publish an empty message.");
            return;
        }
        auto message = std_msgs::msg::String();
        message.data = base_string_;
        RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'",
                    message.data.c_str());
        publisher_->publish(message);
    }

 private:
    /**
     * @brief Handle requests to change the base string
     * @param request The request object containing the new string
     * @param response The response object to convey success status
     */
    void handleChangeStringRequest(const std::shared_ptr
                    <beginner_tutorials::srv::ChangeString::Request>
                    request, std::shared_ptr
                    <beginner_tutorials::srv::ChangeString::Response>
                    response) {
        if (request->new_string.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                        "Received an empty string");
            response->success = false;
            return;
        }
        base_string_ = request->new_string;
        response->success = true;
        RCLCPP_WARN_STREAM(this->get_logger(),
                    "Base string changed to: " << base_string_);
    }

    ///< ROS2 publisher to the "chatter" topic
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    ///< ROS2 service to change the base string
    rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;

    ///< The base string to be published
    std::string base_string_;
};

/**
 * @brief Main function to initialize and run the Talker node
 * @param argc Number of arguments
 * @param argv Argument vector
 * @return int Execution status
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();

    rclcpp::Rate rate(1);  // 1 Hz
    while (rclcpp::ok()) {
        node->publishMessage();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
