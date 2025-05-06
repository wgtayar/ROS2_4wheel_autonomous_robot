#include "robot_nav/nav.h"
#include <fstream>
#include <sstream>

#include <rclcpp/callback_group.hpp>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robot_nav {

// Define the VFFVectors struct
struct VFFVectors {
  std::vector<float> wander;
  std::vector<float> repulsive;
  std::vector<float> result;
};

Define the VFFColor enum
enum VFFColor {
  RED,
  GREEN,
  BLUE
};

WanderNode::WanderNode()
    : Node("nav_wander")
{
  // Initialize random number generator for wandering behavior
  std::random_device rd;
  random_generator_ = std::mt19937(rd());
  
  // Declare parameters
  this->declare_parameter("obstacle_distance", 1.0);     // meters
  this->declare_parameter("max_linear_speed", 0.2);      // m/s
  this->declare_parameter("max_angular_speed", 0.5);     // rad/s
  this->declare_parameter("wander_strength", 0.5);       // scalar multiplier
  this->declare_parameter("wander_rate", 0.05);          // how quickly direction changes
  this->declare_parameter("obstacle_influence_distance", 2.0); // meters
  this->declare_parameter("is_wandering", true);         // enable/disable wandering
  
  // Initialize wander direction with a random angle
  std::uniform_real_distribution<float> dist(0.0, 2.0 * M_PI);
  wander_angle_ = dist(random_generator_);
  
  // Publisher for velocity commands
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  
  // Publisher for debug visualization
  vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 10);
  
  // Subscriber for laser scan data
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", 
    rclcpp::SensorDataQoS(), 
    std::bind(&WanderNode::scan_callback, this, _1));
  
  // Service to toggle wandering on/off
  service_ = this->create_service<std_srvs::srv::SetBool>(
    "toggle_wandering",
    std::bind(&WanderNode::toggle_wandering, this, _1, _2),
    rmw_qos_profile_services_default,
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  
  // Timer for control cycle
  timer_ = create_wall_timer(50ms, std::bind(&WanderNode::control_cycle, this));
  
  // Initialize last_scan_ as nullptr
  last_scan_ = nullptr;
  
  RCLCPP_INFO(this->get_logger(), "Wandering robot node initialized");
}

void WanderNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

void WanderNode::toggle_wandering(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  this->set_parameter(rclcpp::Parameter("is_wandering", request->data));
  
  if (request->data) {
    RCLCPP_INFO(this->get_logger(), "Wandering behavior enabled");
    response->message = "Wandering behavior enabled";
  } else {
    RCLCPP_INFO(this->get_logger(), "Wandering behavior disabled");
    response->message = "Wandering behavior disabled";
    
    // Send stop command when disabling wandering
    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);
  }
  
  response->success = true;
}

void WanderNode::control_cycle()
{
  // Skip if we don't have scan data or wandering is disabled
  if (!last_scan_ || !this->get_parameter("is_wandering").as_bool()) return;
  
  // Laser scan data is older than 1 second, skip
  if ((this->now() - last_scan_->header.stamp) > 1s) return;
  
  // Update wander direction
  update_wander_direction();
  
  // Calculate forces
  VFFVectors vff = calculate_forces(*last_scan_);
  
  // Compute velocity command based on forces
  geometry_msgs::msg::Twist vel = compute_velocity(vff);
  
  // Publish velocity command
  vel_pub_->publish(vel);
  
  // Publish debug visualization if someone is subscribed
  if (vff_debug_pub_->get_subscription_count() > 0) {
    vff_debug_pub_->publish(get_debug_vff(vff));
  }
}

void WanderNode::update_wander_direction()
{
  // Get wander rate from parameter
  float wander_rate = this->get_parameter("wander_rate").as_double();
  
  // Add small random change to wander angle
  std::uniform_real_distribution<float> dist(-wander_rate, wander_rate);
  wander_angle_ += dist(random_generator_);
  
  // Keep angle between 0 and 2π
  while (wander_angle_ < 0) wander_angle_ += 2 * M_PI;
  while (wander_angle_ >= 2 * M_PI) wander_angle_ -= 2 * M_PI;
}

VFFVectors WanderNode::calculate_forces(const sensor_msgs::msg::LaserScan & scan)
{
  float obstacle_distance = this->get_parameter("obstacle_distance").as_double();
  float influence_distance = this->get_parameter("obstacle_influence_distance").as_double();
  float wander_strength = this->get_parameter("wander_strength").as_double();
  
  VFFVectors vff{
    {0.0, 0.0},  // wander
    {0.0, 0.0},  // repulsive
    {0.0, 0.0}   // result
  };
  
  // Calculate wander force
  vff.wander[0] = cos(wander_angle_) * wander_strength;
  vff.wander[1] = sin(wander_angle_) * wander_strength;
  
  // Calculate repulsive force from obstacles
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
      force_magnitude = std::min(force_magnitude, 5.0f);
      
      // Add this obstacle's contribution to the total repulsive force
      vff.repulsive[0] += cos(opposite_angle) * force_magnitude;
      vff.repulsive[1] += sin(opposite_angle) * force_magnitude;
    }
  }
  
  // Normalize the repulsive force if it's too strong
  float repulsive_magnitude = std::sqrt(
    vff.repulsive[0] * vff.repulsive[0] + 
    vff.repulsive[1] * vff.repulsive[1]);
    
  if (repulsive_magnitude > 2.0) {
    vff.repulsive[0] = (vff.repulsive[0] / repulsive_magnitude) * 2.0;
    vff.repulsive[1] = (vff.repulsive[1] / repulsive_magnitude) * 2.0;
  }
  
  // Calculate resultant force
  vff.result[0] = vff.wander[0] + vff.repulsive[0];
  vff.result[1] = vff.wander[1] + vff.repulsive[1];
  
  return vff;
}

geometry_msgs::msg::Twist WanderNode::compute_velocity(const VFFVectors & vff)
{
  geometry_msgs::msg::Twist vel;
  
  // Calculate direction and magnitude of resultant force
  double angle = atan2(vff.result[1], vff.result[0]);
  double magnitude = sqrt(vff.result[0] * vff.result[0] + vff.result[1] * vff.result[1]);
  
  // Get speed limits from parameters
  double max_linear_speed = this->get_parameter("max_linear_speed").as_double();
  double max_angular_speed = this->get_parameter("max_angular_speed").as_double();
  
  // Compute linear and angular velocity
  vel.linear.x = std::clamp(magnitude, 0.0, max_linear_speed);
  vel.angular.z = std::clamp(angle, -max_angular_speed, max_angular_speed);
  
  return vel;
}

visualization_msgs::msg::MarkerArray WanderNode::get_debug_vff(const VFFVectors & vff)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Wander force (blue)
  marker_array.markers.push_back(make_marker(vff.wander, BLUE));
  
  // Repulsive force (red)
  marker_array.markers.push_back(make_marker(vff.repulsive, RED));
  
  // Resultant force (green)
  marker_array.markers.push_back(make_marker(vff.result, GREEN));
  
  return marker_array;
}

visualization_msgs::msg::Marker WanderNode::make_marker(const std::vector<float> & vector, VFFColor color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_footprint";
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Start point of the marker
  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  
  // End point of the marker (force vector)
  geometry_msgs::msg::Point end;
  end.x = vector[0];
  end.y = vector[1];
  end.z = 0.0;
  
  // Set marker points
  marker.points.push_back(start);
  marker.points.push_back(end);
  
  // Set marker scale
  marker.scale.x = 0.05;  // shaft diameter
  marker.scale.y = 0.1;   // head diameter
  
  // Set marker color based on force type
  switch (color) {
    case RED:  // Repulsive force
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.id = 0;
      break;
    case GREEN:  // Resultant force
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.id = 1;
      break;
    case BLUE:  // Wander force
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.id = 2;
      break;
  }
  
  // Set marker transparency (alpha)
  marker.color.a = 1.0;
  
  return marker;
}

} // namespace robot_nav