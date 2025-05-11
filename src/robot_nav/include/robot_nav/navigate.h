#ifndef ROBOT_NAV__NAVIGATE_HPP_
#define ROBOT_NAV__NAVIGATE_HPP_

#include <string>
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <robot_nav/action/navigate.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robot_nav
{

  using NavigateAction = robot_nav::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateAction>;

  struct VFFVectors
  {
    std::array<float, 2> attractive{}, repulsive{}, result{};
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
    // action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);
    void handle_accepted(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);

    // old helpers
    VFFVectors get_vff(const sensor_msgs::msg::LaserScan &scan);
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    double goal_tolerance_, stop_distance_;
    bool wander_{true}, has_goal_{false};
    std::string target_frame_;
    std::array<double, 2> current_pos_{};
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

    // ROS interfaces
    rclcpp_action::Server<NavigateAction>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr cycle_timer_;
  };

} // namespace robot_nav

#endif // ROBOT_NAV__NAVIGATE_HPP_
