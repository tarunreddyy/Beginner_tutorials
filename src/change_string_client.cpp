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
 * @file change_string_client.cpp
 * @author Tarun Trilokesh
 * @date 11/07/2023
 * @version 1.0
 * 
 * @brief Main entry point for the ChangeStringClient node.
 *
 * This program initializes a ROS 2 node as a service client that
 * requests a change in the string being published by the 'talker' node.
 */
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

/**
 * @brief A ChangeStringClient class for ROS2
 * @details This class represents a ROS2 service client node that sends requests to change
 * the message string being published by the 'talker' node.
 */
class ChangeStringClient : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for ChangeStringClient
   * @param service_name The name of the change_string service provided by the 'talker' node.
   */
  explicit ChangeStringClient(const std::string &service_name)
  : Node("change_string_client") {
    client_ = this->create_client<
        beginner_tutorials::srv::ChangeString>(service_name);
  }

  /**
   * @brief Send a request to change the published string
   * @param new_string The new string to be published by the 'talker' node
   */
  void send_request(const std::string &new_string) {
    auto request = std::make_shared<
        beginner_tutorials::srv::ChangeString::Request>();
    request->new_string = new_string;

    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }

    // Send the request asynchronously
    auto result = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed!");
    }
  }

 private:
  ///< Client object for the change_string service
  rclcpp::Client<beginner_tutorials::srv::ChangeString>::SharedPtr client_;
};

/**
 * @brief Main function to initialize and run the ChangeStringClient node
 * @param argc Number of command-line arguments
 * @param argv Vector of command-line arguments
 * @return int Execution status code
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChangeStringClient>("change_string");

    // Check command-line arguments for the new string
    if (argc != 2) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: change_string_client new_string");
        return 1;
    }
    std::string new_string = argv[1];
    node->send_request(new_string);

    // Spin the node to handle callbacks
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
