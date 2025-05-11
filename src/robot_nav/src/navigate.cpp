#include "robot_nav/navigate.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

using namespace std::placeholders;
namespace rn = robot_nav;

rn::Navigate::Navigate()
    : Node("navigate_server"),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.1);
  stop_distance_ = this->declare_parameter("stop_distance", 0.2);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Navigate::scan_callback, this, _1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&Navigate::odom_callback, this, _1));
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 10);

  cycle_timer_ = create_wall_timer(
      50ms, [this]()
      {
      if (wander_) {
        auto cmd = geometry_msgs::msg::Twist{};
        cmd.linear.x = 0.1;
        vel_pub_->publish(cmd);
      } });

  action_server_ = rclcpp_action::create_server<NavigateAction>(
      this, "navigate",
      std::bind(&Navigate::handle_goal, this, _1, _2),
      std::bind(&Navigate::handle_cancel, this, _1),
      std::bind(&Navigate::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse
rn::Navigate::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const NavigateAction::Goal> goal)
{
  wander_ = goal->wander;
  target_frame_ = goal->target_frame;
  has_goal_ = !wander_ && !target_frame_.empty();
  RCLCPP_INFO(get_logger(),
              "[navigate] Received goal: wander=%s, target=%s",
              wander_ ? "true" : "false",
              target_frame_.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
rn::Navigate::handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> /*gh*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void rn::Navigate::handle_accepted(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{[this, goal_handle]()
              { this->execute(goal_handle); }}
      .detach();
}

void rn::Navigate::execute(
    const std::shared_ptr<GoalHandleNavigate> gh)
{
  rclcpp::Rate loop(20);
  auto result = std::make_shared<NavigateAction::Result>();

  while (rclcpp::ok())
  {
    if (gh->is_canceling())
    {
      gh->canceled(result);
      return;
    }

    float distance = 0.0F;
    if (has_goal_)
    {
      try
      {
        auto tf = tf_buffer_.lookupTransform(
            "dummy_link", target_frame_,
            tf2::TimePointZero);
        double dx = tf.transform.translation.x;
        double dy = tf.transform.translation.y;
        distance = std::hypot(dx, dy);

        geometry_msgs::msg::Twist cmd;
        if (distance < stop_distance_)
        {
          cmd = geometry_msgs::msg::Twist{};
        }
        else
        {
          double th = std::atan2(dy, dx);
          cmd.linear.x = std::min(0.3, distance);
          cmd.angular.z = std::clamp(th, -0.5, 0.5);
        }
        vel_pub_->publish(cmd);
      }
      catch (...)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "TF lookup failed for %s", target_frame_.c_str());
      }
    }

    auto fb = std::make_shared<NavigateAction::Feedback>();
    fb->distance_to_goal = distance;
    fb->arrived = (distance < goal_tolerance_);
    gh->publish_feedback(fb);

    if (fb->arrived)
    {
      result->success = true;
      result->message = "arrived";
      gh->succeed(result);
      return;
    }
    loop.sleep();
  }
}

void rn::Navigate::scan_callback(
    sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

void rn::Navigate::odom_callback(
    nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pos_[0] = msg->pose.pose.position.x;
  current_pos_[1] = msg->pose.pose.position.y;
}
