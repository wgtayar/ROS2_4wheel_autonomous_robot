#ifndef ROBOT_NAV_NAV_H
#define ROBOT_NAV_NAV_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <random>

namespace robot_nav {

// Forward declarations
struct VFFVectors {
    std::vector<float> wander;
    std::vector<float> repulsive;
    std::vector<float> result;
};
  
enum VFFColor {
    RED,
    GREEN,
    BLUE
};
  

class WanderNode : public rclcpp::Node
{
public:
  WanderNode();

private:
  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void toggle_wandering(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void control_cycle();

  // Methods
  void update_wander_direction();
  VFFVectors calculate_forces(const sensor_msgs::msg::LaserScan & scan);
  geometry_msgs::msg::Twist compute_velocity(const VFFVectors & vff);
  visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors & vff);
  visualization_msgs::msg::Marker make_marker(const std::vector<float> & vector, VFFColor color);

  // Publishers, subscribers, services
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  float wander_angle_;
  std::mt19937 random_generator_;
};

} // namespace robot_nav

#endif  // ROBOT_NAV_NAV_H