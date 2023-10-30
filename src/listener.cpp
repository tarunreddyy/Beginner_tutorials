// Copyright 2023 Tarun Trilokesh

/**
 * @file listener.cpp
 * @author Tarun Trilokesh
 * @date 10/29/2023
 * @version 2.0
 *
 * @brief Main entry point for the beginner_tutorials subscriber node.
 *
 * This program initializes a ROS 2 node, creates a subscriber,
 * and subscribes to the "chatter" topic to receive string messages.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

/**
 * @class Talker
 * @brief ROS2 node that publishes messages and provides a service to change the base string.
 */
class Talker : public rclcpp::Node {
 public:
    Talker() : Node("talker"), base_string_("Hello World!") {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "chatter", 10);
        service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
            "change_string", std::bind(&Talker::handleChangeStringRequest, this,
            std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Talker node initialized with base string: "
                           << base_string_);
    }

    void publishMessage() {
        auto message = std_msgs::msg::String();
        message.data = base_string_;
        RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'",
            message.data.c_str());
        publisher_->publish(message);
    }

 private:
    void handleChangeStringRequest(
        const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
            request,
        std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
            response) {
        base_string_ = request->new_string;
        response->success = true;
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Base string changed to: " << base_string_);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
    std::string base_string_;
};

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
