#include "rclcpp/publisher_options.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

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
        "/fastbot_1/scan", 10,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1),
        sub_options);

    timer_action_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::determine_action_callback, this),
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
    // Find the minimum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;
      bool is_front = (sector.first.find("Front") != std::string::npos);

      // Ensure the index range is within bounds
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        auto start_it = msg->ranges.begin() + start_idx;
        auto end_it = msg->ranges.begin() + end_idx + 1;
        auto sector_max = 0;
        auto range_min = msg->range_min;
        auto range_max = msg->range_max;

        if (start_it < end_it) {
          for (auto it = start_it; it != end_it; ++it) {
            if (!std::isfinite(*it)) {
              continue;
            } else if (not(*it > range_min)) {
              continue;
            } else if (not(*it < range_max)) {
              continue;
            }
            // if it is a front sector and the closest we've seen, save it
            if (*it < min_distances[sector.first]) {
              min_distances[sector.first] = *it;
            }
            if (*it > sector_max) {
              sector_max = *it;
            }
          }

          if (sector_max > overall_max_distance) {
            // overall max distance in this sector, so if we are blocked
            // then this is the direction to turn
            safest_direction = sector.first;
            overall_max_distance = sector_max;
          }
          // only update overall_min_distance if it's a front sector
          if (is_front &&
              (min_distances[sector.first] < overall_min_distance)) {
            overall_min_distance = min_distances[sector.first];
          }
        }
        RCLCPP_INFO(this->get_logger(),
                    ("sector min" + sector.first +
                     std::to_string(min_distances[sector.first]))
                        .c_str());
      }
      RCLCPP_INFO(this->get_logger(),
                  ("overall min" + std::to_string(overall_min_distance) +
                   ", safest " + safest_direction)
                      .c_str());
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
      RCLCPP_INFO(this->get_logger(),
                  ("Obstacle " + std::to_string(overall_min_distance) +
                   " ahead, turning to left.")
                      .c_str());
    } else {
      action.linear.x = 0.0;
      action.angular.z = -0.3; // Turn right
      RCLCPP_INFO(this->get_logger(),
                  ("Obstacle " + std::to_string(overall_min_distance) +
                   " ahead, turning to right.")
                      .c_str());
    }

    publisher_->publish(action);
  }

private:
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_1_;
  rclcpp::CallbackGroup::SharedPtr mutuallyexclusive_group_2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::TimerBase::SharedPtr timer_action_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  float distance_from_obstacle_;
  std::map<std::string, std::pair<int, int>> sectors = {
      {"Right", {12, 50}},
      {"Front_Right", {1, 11}},
      {"Front_Left", {189, 200}},
      {"Left", {150, 188}}};
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