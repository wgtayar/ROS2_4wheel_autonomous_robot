#ifndef ROBOT_NAV_NAV2GOAL_H
#define ROBOT_NAV_NAV2GOAL_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "robot_nav/srv/goal_input.hpp"  // Custom service definition

namespace robot_nav {

// Forward declarations
// Define the VFFVectors struct
struct VFFVectors {
    std::vector<float> attractive;
    std::vector<float> repulsive;
    std::vector<float> result;
};
  
enum VFFColor {
    RED,
    GREEN,
    BLUE
};

class NavigateToGoal : public rclcpp::Node
{
public:
  NavigateToGoal();

private:
  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handle_navigate_to_goal(
    const std::shared_ptr<robot_nav::srv::GoalInput::Request> request,
    std::shared_ptr<robot_nav::srv::GoalInput::Response> response);
  void control_cycle();

  // Methods
  geometry_msgs::msg::Twist compute_vff_velocity();
  double calculate_distance_to_goal();
  VFFVectors get_vff(const sensor_msgs::msg::LaserScan & scan);
  visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff_vectors);
  visualization_msgs::msg::Marker make_marker(const std::vector<float> & vector, VFFColor color);

  // Publishers, subscribers, services
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<robot_nav::srv::GoalInput>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // State variables
  std::vector<float> target_;
  std::vector<float> current_pos_;
  bool has_goal_;
  bool goal_reached_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

} // namespace robot_nav

#endif  // ROBOT_NAV_NAV2GOAL_H