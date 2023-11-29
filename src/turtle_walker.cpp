/**
 * @file turtle_walker.cpp
 * @author Vinay Bukka (vinay06@umd.edu)
 * @brief This program is used to make a turtlebot move in Gazebo by avoiding
 * obstacles. This is achieved through lidar scan.
 * @version 0.1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief The Class Initiates the constructor with publisher and subscriber.
 *
 */
class Turtlebot_Walker : public rclcpp::Node {
 public:
  Turtlebot_Walker() : Node("turtlebot_walker"), count_(0) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 5,
            std::bind(&Turtlebot_Walker::laser_scan_callback, this, _1));
  }

 private:
  /**
   * @brief The call back function updates the message from scan to detect the
   * obstacle
   *
   * @param msg The variable msg contains the lidar scan range. Based on which
   * the obstacle in front of robot is calculated with some threshold
   */
  void laser_scan_callback(const sensor_msgs::msg::LaserScan& msg) {
    if (msg.header.stamp.sec == 0) {
      return;
    }
    auto scan_data = msg.ranges;
    auto field_range = 60;
    auto initial_angle = 330;
    bool obstacle_detected = false;

    for (int i = initial_angle; i < initial_angle + field_range; i++) {
      if (scan_data[i % 360] < 0.75) {
        obstacle_detected = true;
        break;
      }
    }
    if (obstacle_detected) {
      rotate_inplace(0.3);

    } else {
      move_forward(0.5);
    }
  }
  /**
   * @brief This function is called whenever robot needs to move forward if no
   * obstacle
   *
   * @param translate_velocity The velocity in x direction which is
   * translational
   */
  void move_forward(double translate_velocity) {
    velocity_msg.linear.x = translate_velocity;
    velocity_msg.angular.z = 0.0;
    publisher_->publish(velocity_msg);
  }
  /**
   * @brief This function makes the robot to rotate in anticlockwise if an
   * obstacle is present
   *
   * @param rotation_velocity The rate at which the robot should rotate
   */
  void rotate_inplace(double rotation_velocity) {
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = rotation_velocity;
    publisher_->publish(velocity_msg);
  }
  geometry_msgs::msg::Twist velocity_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  size_t count_;
};
/**
 * @brief Main function calling the Turtlebot Node
 *
 * @param argc Number of arguments
 * @param argv Argument list
 * @return int Dummy value return once the node stops
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot_Walker>());
  rclcpp::shutdown();
  return 0;
}
