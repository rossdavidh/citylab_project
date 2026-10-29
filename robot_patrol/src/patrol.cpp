#include "rclcpp/publisher_options.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    // Create two mutually exclusive callback groups
    mutuallyexclusive_group_1_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    mutuallyexclusive_group_2_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Set up laserscan for first callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mutuallyexclusive_group_1_;
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1),
        sub_options);

    timer_action_ = this->create_wall_timer(
        0.1, std::bind(&Patrol::determine_action_callback, this),
        mutuallyexclusive_group_2_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    // Initialize state variables
    distance_from_obstacle_ = std::numeric_limits<float>::infinity();

    RCLCPP_INFO(this->get_logger(), "Patrol Node Ready...");
  }

private:
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Initialize the minimum distances for each sector
    std::map<std::string, float> min_distances;
    for (const auto &sector : sectors) {
      min_distances[sector.first] = std::numeric_limits<float>::infinity();
    }
    safest_direction = "";
    overall_min_distance = std::numeric_limits<float>::infinity();
    float overall_max_distance = 0;
    int max_index;
    // Find the minimum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      // Ensure the index range is within bounds
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        auto start_it = msg->ranges.begin() + start_idx;
        auto end_it = msg->ranges.begin() + end_idx + 1;

        if (start_it < end_it) {
          min_distances[sector.first] = *std::min_element(start_it, end_it);
          if (*std::max_element(start_it, end_it) > overall_max_distance) {
            max_index = std::distance(start_it, overall_max_distance);
            safest_direction = sector.first;
          }
        }
        if (min_distances[sector.first] < overall_min_distance) {
          overall_min_distance = min_distances[sector.first];
        }
        // TODO: find direction
      }
    }
  }
  void determine_action_callback() {
    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.35; // meters

    // Determine suggested action based on detection
    auto action = geometry_msgs::msg::Twist();
    auto clear_path = overall_min_distance > obstacle_threshold;

    // If an obstacle is detected in either front sector, continue turning
    if (clear_path) {
      action.linear.x = 0.1; // Move forward
      action.angular.z = 0.0;
      // RCLCPP_INFO(this->get_logger(), "No obstacles, moving forward.");
    } else if (safest_direction == "Front_Left") {
      action.linear.x = 0.0;
      action.angular.z = 0.3; // Turn left
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to left.");
    } else {
      action.linear.x = 0.0;
      action.angular.z = -0.3; // Turn right
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead, turning to right.");
    }

    publisher_->publish(action);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::TimerBase::SharedPtr timer_action_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float distance_from_obstacle_;
  std::map<std::string, std::pair<int, int>> sectors = {
      {"Front_Right", {75, 100}}, {"Front_Left", {101, 125}}};
  float overall_min_distance;
  std::string safest_direction;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}