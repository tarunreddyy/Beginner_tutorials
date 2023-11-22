// Copyright 2023 Tarun Trilokesh
//
// Licensed under the MIT License (MIT); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file talker.cpp
 * @author Tarun Trilokesh
 * @date 11/17/2023
 * @version 3.0
 * 
 * @brief Main entry point for the beginner_tutorials publisher node.
 *
 * This program initializes a ROS 2 node, creates a publisher,
 * publishes a custom string message to the "chatter" topic, and
 * broadcasts a TF frame named /talk with the parent /world.
 */

// C++ system headers
#include <memory>

// Other libraries' headers
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

/**
 * @class Talker
 * @brief A ROS2 publisher node class
 * 
 * The Talker class is responsible for initializing a ROS2 node,
 * creating a publisher to the "chatter" topic, broadcasting a TF frame,
 * and providing a service to change the message string.
 */
class Talker : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for the Talker class
     * 
     * Initializes the ROS2 publisher, service, TF broadcaster, and sets the base string.
     */
    Talker();

    /**
     * @brief Publish a message and broadcast a TF frame
     * 
     * This method publishes the current base string as a ROS2 message,
     * and broadcasts a static transform between /world and /talk.
     */
    void publishMessage();

 private:
    /**
     * @brief Handle requests to change the base string
     * @param request The request object containing the new string
     * @param response The response object to convey success status
     * 
     * Updates the base string with the new value provided in the request.
     * Sets the response success flag accordingly.
     */
    void handleChangeStringRequest(
        const std::shared_ptr<beginner_tutorials::srv
        ::ChangeString::Request> request,
        std::shared_ptr<beginner_tutorials::srv
        ::ChangeString::Response> response);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string base_string_;
};

Talker::Talker() : Node("talker"), base_string_("Hello World!") {
    /**
     * @brief Constructor implementation for the Talker class
     * 
     * Initializes the ROS2 publisher, service, TF broadcaster, and sets the base string.
     */
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
                "change_string",
                std::bind(&Talker::handleChangeStringRequest,
                this,
                std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO_STREAM(this->get_logger(),
                        "Talker node initialized with base string: "
                        << base_string_);
}

void Talker::publishMessage() {
    /**
     * @brief Publish a message and broadcast a TF frame
     * 
     * This method publishes the current base string as a ROS2 message,
     * and broadcasts a static transform between /world and /talk.
     */
    if (base_string_.empty()) {
        RCLCPP_FATAL(this->get_logger(),
        "Base string is empty. Cannot publish an empty message.");
        return;
    }
    auto message = std_msgs::msg::String();
    message.data = base_string_;
    RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    // Broadcasting a TF frame /talk with parent /world
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "talk";
    // Set non-zero translation and rotation
    transformStamped.transform.translation.x = 1.0;  // Example values
    transformStamped.transform.translation.y = 2.0;
    transformStamped.transform.translation.z = 3.0;
    tf2::Quaternion q;
    q.setRPY(1.0, 0.5, 0.0);  // Example rotation values in radians
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
}

void Talker::handleChangeStringRequest(
    const std::shared_ptr<beginner_tutorials::srv
                                    ::ChangeString::Request> request,
    std::shared_ptr<beginner_tutorials::srv
                                    ::ChangeString::Response> response) {
    /**
     * @brief Handle requests to change the base string
     * @param request The request object containing the new string
     * @param response The response object to convey success status
     * 
     * Updates the base string with the new value provided in the request.
     * Sets the response success flag accordingly.
     */
    if (request->new_string.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received an empty string");
        response->success = false;
        return;
    }
    base_string_ = request->new_string;
    response->success = true;
    RCLCPP_WARN_STREAM(this->get_logger(),
                        "Base string changed to: " << base_string_);
}

/**
 * @brief Main function to initialize and run the Talker node
 * @param argc Number of arguments
 * @param argv Argument vector
 * @return int Execution status
 * 
 * Initializes the ROS2 system, creates a Talker object, and keeps
 * spinning until ROS2 shutdown is requested.
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
