#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Turtlebot_Walker : public rclcpp::Node {
 public:
  Turtlebot_Walker() : Node("turtlebot_walker"), count_(0) {
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
              "/scan", 5, std::bind(&Turtlebot_Walker::laser_scan_callback, this, _1));
    
  }

 private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    if(msg.header.stamp.sec == 0){
      return;
    }
    auto scan_data = msg.ranges;
    auto field_range = 60;
    auto initial_angle = 330;
    bool obstacle_detected = false;

    for (i=initial_angle;i<initial_angle+field_range;i++){
      if(scan_data[i%360] < 0.75){
        obstacle_detected = true;
        break;
      }
    }
    if(obstacle_detected){
      rotate_inplace(0.3);

    } else {
      move_forward(0.5);
    }
  }
  void move_forward(double translate_velocity){
    velocity_msg.linear.x = translate_velocity;
    velocity_msg.angular.z = 0.0;
    publisher_ ->publish(velocity_msg)
  }
  void rotate_inplace(double rotation_velocity){
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = rotation_velocity;
    publisher_ ->publish(velocity_msg)
  }
  geometry_msgs::msg::Twist velocity_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  size_t count_;
 
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot_Walker>());
  rclcpp::shutdown();
  return 0;
}
