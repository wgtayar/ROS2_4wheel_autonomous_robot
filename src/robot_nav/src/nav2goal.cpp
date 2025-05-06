#include "robot_nav/nav2goal.h"
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/callback_group.hpp>

#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robot_nav {

// Define the VFFVectors struct
struct VFFVectors {
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
};

// Define the VFFColor enum
// enum VFFColor {
//   RED,
//   GREEN,
//   BLUE
// };

NavigateToGoal::NavigateToGoal()
    : Node("nav2goal")
{
  // Initialize member variables
  target_ = {0.0, 0.0, 0.0};
  current_pos_ = {0.0, 0.0, 0.0};
  has_goal_ = false;
  goal_reached_ = false;
  last_scan_ = nullptr;
  
  // Declare parameters
  this->declare_parameter("goal_tolerance", 0.1);  // meters
  this->declare_parameter("stop_distance", 0.2);   // meters before goal
  this->declare_parameter("obstacle_distance", 1.0); // meters
  this->declare_parameter("max_linear_speed", 0.3);  // m/s
  this->declare_parameter("max_angular_speed", 0.5); // rad/s
  this->declare_parameter("obstacle_influence_distance", 2.0); // meters

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 10);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", 
    rclcpp::SensorDataQoS(), 
    std::bind(&NavigateToGoal::scan_callback, this, _1));
    
  // Subscribe to odometry or pose topic to get current position
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "robot_pose", 
    10, 
    std::bind(&NavigateToGoal::pose_callback, this, _1));

  service_ = this->create_service<robot_nav::srv::GoalInput>(
    "navigate_to_goal",
    std::bind(&NavigateToGoal::handle_navigate_to_goal, this, _1, _2));

  timer_ = create_wall_timer(50ms, std::bind(&NavigateToGoal::control_cycle, this));
}

void NavigateToGoal::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

void NavigateToGoal::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pos_[0] = msg->pose.position.x;
  current_pos_[1] = msg->pose.position.y;
  current_pos_[2] = msg->pose.position.z;
}

void NavigateToGoal::handle_navigate_to_goal(
  const std::shared_ptr<robot_nav::srv::GoalInput::Request> request,
  std::shared_ptr<robot_nav::srv::GoalInput::Response> response)
{
  target_[0] = request->x;
  target_[1] = request->y;
  target_[2] = request->z;
  has_goal_ = true;
  goal_reached_ = false;  // Reset goal reached flag for the new goal
  response->success = true;
  
  double distance = calculate_distance_to_goal();
  double stop_distance = this->get_parameter("stop_distance").as_double();
  
  RCLCPP_INFO(this->get_logger(), "Received new goal: x=%.2f, y=%.2f, z=%.2f", 
              target_[0], target_[1], target_[2]);
  RCLCPP_INFO(this->get_logger(), "Current distance to goal: %.2f meters (will stop %.2f meters before goal)",
              distance, stop_distance);
}

void NavigateToGoal::control_cycle()
{
  if (!last_scan_ || !has_goal_) return;
  
  // Check if we've reached the goal
  if (!goal_reached_) {
    double goal_tolerance = this->get_parameter("goal_tolerance").as_double();
    double distance_to_goal = calculate_distance_to_goal();
    
    if (distance_to_goal <= goal_tolerance) {
      RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
      goal_reached_ = true;
      
      // Send stop command
      geometry_msgs::msg::Twist stop_cmd;
      vel_pub_->publish(stop_cmd);
      return;
    }
    
    // If we're still going to the goal, compute and publish velocity
    geometry_msgs::msg::Twist vel = compute_vff_velocity();
    vel_pub_->publish(vel);
  }
}

double NavigateToGoal::calculate_distance_to_goal()
{
  // Calculate Euclidean distance between current position and target
  double dx = target_[0] - current_pos_[0];
  double dy = target_[1] - current_pos_[1];
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Twist NavigateToGoal::compute_vff_velocity()
{
  geometry_msgs::msg::Twist vel;

  if ((this->now() - last_scan_->header.stamp) > 1s) return vel;

  VFFVectors vff = get_vff(*last_scan_);

  // Compute attractive force toward target
  double distance_to_goal = calculate_distance_to_goal();
  double stop_distance = this->get_parameter("stop_distance").as_double();
  
  // Direction vector to goal
  double dx = target_[0] - current_pos_[0];
  double dy = target_[1] - current_pos_[1];
  
  // Normalize the direction
  double norm = std::sqrt(dx * dx + dy * dy);
  if (norm > 0.001) {  // Avoid division by zero
    dx /= norm;
    dy /= norm;
  }
  
  // If we're close to the goal, adjust attractive force to stop at the desired distance
  if (distance_to_goal > stop_distance) {
    // Regular attractive force
    vff.attractive[0] = dx;
    vff.attractive[1] = dy;
  } else {
    // We want to stop at stop_distance from goal, so reduce attractive force
    double stop_factor = (distance_to_goal / stop_distance) * 0.5;
    vff.attractive[0] = dx * stop_factor;
    vff.attractive[1] = dy * stop_factor;
  }

  vff.result[0] = vff.repulsive[0] + vff.attractive[0];
  vff.result[1] = vff.repulsive[1] + vff.attractive[1];

  double angle = atan2(vff.result[1], vff.result[0]);
  double module = sqrt(vff.result[0] * vff.result[0] + vff.result[1] * vff.result[1]);

  double max_linear_speed = this->get_parameter("max_linear_speed").as_double();
  double max_angular_speed = this->get_parameter("max_angular_speed").as_double();
  
  vel.linear.x = std::clamp(module, 0.0, max_linear_speed);
  vel.angular.z = std::clamp(angle, -max_angular_speed, max_angular_speed);

  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }

  return vel;
}

VFFVectors NavigateToGoal::get_vff(const sensor_msgs::msg::LaserScan & scan)
{
  float obstacle_distance = this->get_parameter("obstacle_distance").as_double();
  float influence_distance = this->get_parameter("obstacle_influence_distance").as_double();
  
  VFFVectors vff_vector{
    {0.0, 0.0},  // attractive
    {0.0, 0.0},  // repulsive
    {0.0, 0.0}   // result
  };

  // Process all ranges to consider multiple obstacles
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    float range = scan.ranges[i];
    
    // Skip invalid readings or readings beyond our influence distance
    if (!std::isfinite(range) || range > influence_distance) {
      continue;
    }
    
    // Calculate repulsive force from this obstacle
    if (range < obstacle_distance) {
      float angle = scan.angle_min + scan.angle_increment * i;
      float opposite_angle = angle + M_PI;
      
      // The closer the obstacle, the stronger the repulsive force
      // Use inverse square law for force magnitude
      float force_magnitude = obstacle_distance / (range * range);
      
      // Cap the maximum force magnitude for very close obstacles
      force_magnitude = std::min(force_magnitude, 3.0f);
      
      // Add this obstacle's contribution to the total repulsive force
      vff_vector.repulsive[0] += cos(opposite_angle) * force_magnitude;
      vff_vector.repulsive[1] += sin(opposite_angle) * force_magnitude;
    }
  }
  
  // Normalize the repulsive force if it's too strong
  float repulsive_magnitude = std::sqrt(
    vff_vector.repulsive[0] * vff_vector.repulsive[0] + 
    vff_vector.repulsive[1] * vff_vector.repulsive[1]);
    
  if (repulsive_magnitude > 2.0) {
    vff_vector.repulsive[0] = (vff_vector.repulsive[0] / repulsive_magnitude) * 2.0;
    vff_vector.repulsive[1] = (vff_vector.repulsive[1] / repulsive_magnitude) * 2.0;
  }

  return vff_vector;
}

visualization_msgs::msg::MarkerArray NavigateToGoal::get_debug_vff(const VFFVectors & vff_vectors)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
  marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
  marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));
  return marker_array;
}

// visualization_msgs::msg::Marker NavigateToGoal::make_marker(const std::vector<float> & vector, VFFColor color)
// {
//   visualization_msgs::msg::Marker marker;
//   marker.header.frame_id = "base_footprint";
//   marker.header.stamp = this->now();
//   marker.type = visualization_msgs::msg::Marker::ARROW;
//   marker.action = visualization_msgs::msg::Marker::ADD;

//   geometry_msgs::msg::Point start;
//   start.x = 0.0;
//   start.y = 0.0;
//   start.z = 0.0;
  
//   geometry_msgs::msg::Point end;
//   end.x = vector[0];
//   end.y = vector[1];
//   end.z = 0.0;
  
//   marker.points.push_back(start);
//   marker.points.push_back(end);

//   marker.scale.x = 0.05;
//   marker.scale.y = 0.1;

//   switch (color) {
//     case RED: 
//       marker.color.r = 1.0; 
//       marker.color.g = 0.0;
//       marker.color.b = 0.0;
//       marker.id = 0; 
//       break;
//     case GREEN: 
//       marker.color.r = 0.0;
//       marker.color.g = 1.0; 
//       marker.color.b = 0.0;
//       marker.id = 1; 
//       break;
//     case BLUE: 
//       marker.color.r = 0.0;
//       marker.color.g = 0.0;
//       marker.color.b = 1.0; 
//       marker.id = 2; 
//       break;
//   }

//   marker.color.a = 1.0;
//   return marker;
// }

} // namespace robot_nav