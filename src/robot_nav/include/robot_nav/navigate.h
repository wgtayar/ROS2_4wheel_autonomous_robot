#ifndef ROBOT_NAV__NAVIGATE_HPP_
#define ROBOT_NAV__NAVIGATE_HPP_

#include <array>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <robot_nav/srv/navigate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robot_nav
{

  struct VFFVectors
  {
    std::vector<float> attractive;
    std::vector<float> repulsive;
    std::vector<float> result;
  };

  enum VFFColor
  {
    RED,
    GREEN,
    BLUE
  };

  class Navigate : public rclcpp::Node
  {
  public:
    Navigate();

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void handle_navigate(
        const std::shared_ptr<robot_nav::srv::Navigate::Request> request,
        std::shared_ptr<robot_nav::srv::Navigate::Response> response);
    void control_cycle();
    double calculate_distance_to_goal(double tx, double ty);
    VFFVectors get_vff(const sensor_msgs::msg::LaserScan &scan);
    visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors &vff);
    visualization_msgs::msg::Marker make_marker(
        const std::vector<float> &vector,
        VFFColor color);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<robot_nav::srv::Navigate>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string world_frame_;
    bool wander_;
    std::string target_frame_;
    bool has_goal_;
    bool goal_reached_;
    std::array<double, 3> current_pos_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    double goal_tolerance_;
    double stop_distance_;
    double obstacle_distance_;
    double influence_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
  };

}

#endif // ROBOT_NAV__NAVIGATE_HPP_