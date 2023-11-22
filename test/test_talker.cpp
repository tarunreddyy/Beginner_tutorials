// Copyright 2023 Tarun Trilokesh
//
// Licensed under the MIT License (MIT); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_talker.cpp
 * @author Tarun Trilokesh
 * @date 11/17/2023
 * @version 1.0
 * 
 * @brief Unit tests for the Talker node in the beginner_tutorials package.
 *
 * This file contains the tests for the Talker node, ensuring it correctly publishes
 * messages to the "chatter" topic and broadcasts a TF frame named /talk with
 * the parent /world.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @class TestTalkerNode
 * @brief Test fixture for the Talker node.
 * 
 * Sets up a ROS2 node and a subscriber to test the Talker node's message publishing and TF broadcasting.
 */
class TestTalkerNode : public testing::Test {
protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool hasData_ = false;
    bool hasTransform_ = false;

    void SetUp() override {
        rclcpp::init(0, nullptr); // Ensure ROS2 is initialized for each test
        node_ = std::make_shared<rclcpp::Node>("talker_test_node");
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "chatter", 10,
            [this](const std_msgs::msg::String& msg) {
                RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
                hasData_ = true;
            });

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void TearDown() override {
        subscription_.reset(); // Reset the subscription
        tf_listener_.reset();  // Reset the transform listener
        node_.reset();         // Reset the node
        rclcpp::shutdown();    // Shutdown ROS2
    }
};

/**
 * @brief Tests whether the Talker node publishes messages.
 * 
 * Ensures that the Talker node publishes messages to the "chatter" topic.
 */
TEST_F(TestTalkerNode, TalkerPublishesMessage) {
    auto start = node_->now();
    while ((node_->now() - start) < 5s && !hasData_) {
        rclcpp::spin_some(node_);
    }

    EXPECT_TRUE(hasData_);
}

/**
 * @brief Tests whether the Talker node broadcasts a TF frame.
 * 
 * Ensures that the Talker node broadcasts a TF frame named /talk with the parent /world.
 */
TEST_F(TestTalkerNode, TalkerBroadcastsTransform) {
    try {
        auto start = node_->now();
        while ((node_->now() - start) < 5s && !hasTransform_) {
            try {
                auto transform = tf_buffer_->lookupTransform("world", "talk", tf2::TimePointZero, 100ms); // Add timeout
                if (transform.transform.translation.x != 0.0 ||
                    transform.transform.translation.y != 0.0 ||
                    transform.transform.translation.z != 0.0) {
                    hasTransform_ = true;
                }
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node_->get_logger(), "TF Exception: %s", ex.what());
                // Don't throw an exception here, just log it
            }
            rclcpp::spin_some(node_);
        }
    } catch (const std::exception& e) {
        FAIL() << "Unexpected exception caught: " << e.what();
    }

    EXPECT_TRUE(hasTransform_);
}

/**
 * @brief Main function for running the tests.
 * @param argc Number of arguments
 * @param argv Argument vector
 * @return int Execution status of the tests
 * 
 * Initializes the ROS2 system, runs the tests, and shuts down ROS2.
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}