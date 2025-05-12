#ifndef ROBOT_NAV__NAVIGATE_HPP_
#define ROBOT_NAV__NAVIGATE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_nav/action/navigate.hpp"

namespace robot_nav
{

  struct VFFVectors
  {
    std::vector<float> attractive, repulsive, result;
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
    using NavigateAction = robot_nav::action::Navigate;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateAction>;

    Navigate();

  private:
    // action callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const NavigateAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);

    // sensor callbacks & control loop
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_cycle();

    double calculate_distance_to_goal(double tx, double ty);
    VFFVectors get_vff(const sensor_msgs::msg::LaserScan &scan);
    visualization_msgs::msg::MarkerArray get_debug_vff(const VFFVectors &vff);
    visualization_msgs::msg::Marker make_marker(const std::vector<float> &vector, VFFColor color);

    // ROS interfaces
    rclcpp_action::Server<NavigateAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vff_debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // state
    std::string world_frame_, target_frame_;
    bool wander_, has_goal_, goal_reached_;
    std::array<double, 2> current_pos_{0.0, 0.0};
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // parameters
    double goal_tolerance_, stop_distance_;
    double obstacle_distance_, influence_distance_;
    double max_linear_speed_, max_angular_speed_;
  };

} // namespace robot_nav

#endif // ROBOT_NAV__NAVIGATE_HPP_