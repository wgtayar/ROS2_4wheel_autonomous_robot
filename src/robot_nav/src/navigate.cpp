#include "robot_nav/navigate.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace robot_nav
{

  Navigate::Navigate()
      : Node("navigate_server"),
        tf_buffer_(this->get_clock()),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
  {
    world_frame_ = this->declare_parameter("world_frame", std::string("odom"));
    goal_tolerance_ = this->declare_parameter("goal_tolerance", 1.0);
    stop_distance_ = this->declare_parameter("stop_distance", 1.0);
    obstacle_distance_ = this->declare_parameter("obstacle_distance", 1.0);
    influence_distance_ = this->declare_parameter("obstacle_influence_distance", 1.2);
    max_linear_speed_ = this->declare_parameter("max_linear_speed", 0.3);
    max_angular_speed_ = this->declare_parameter("max_angular_speed", 0.5);

    wander_ = true;
    has_goal_ = false;
    goal_reached_ = false;

    action_server_ = rclcpp_action::create_server<NavigateAction>(
        this,
        "navigate",
        std::bind(&Navigate::handle_goal, this, _1, _2),
        std::bind(&Navigate::handle_cancel, this, _1),
        std::bind(&Navigate::handle_accepted, this, _1));

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    vff_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&Navigate::scan_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&Navigate::odom_callback, this, _1));

    timer_ = create_wall_timer(50ms, std::bind(&Navigate::control_cycle, this));
  }

  rclcpp_action::GoalResponse
  Navigate::handle_goal(const rclcpp_action::GoalUUID &,
                        std::shared_ptr<const NavigateAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
                "Received Navigate goal: wander=%s, target='%s'",
                goal->wander ? "true" : "false",
                goal->target_frame.c_str());
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  Navigate::handle_cancel(const std::shared_ptr<GoalHandleNavigate> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Navigate goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void Navigate::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    std::thread{[this, goal_handle]()
                { execute(goal_handle); }}
        .detach();
  }

  void Navigate::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    wander_ = goal->wander;
    target_frame_ = goal->target_frame;
    has_goal_ = (!wander_ && !target_frame_.empty());
    goal_reached_ = false;

    auto feedback = std::make_shared<NavigateAction::Feedback>();
    auto result = std::make_shared<NavigateAction::Result>();

    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        goal_handle->canceled(result);
        return;
      }

      if (goal_reached_)
      {
        RCLCPP_INFO(get_logger(), "Action: detected goal_reached_ --> Succeeding");
        result->success = true;
        result->message = "Goal reached via VFF";
        goal_handle->succeed(result);
        return;
      }

      double dist = std::numeric_limits<double>::infinity();
      if (has_goal_)
      {
        try
        {
          auto t = tf_buffer_.lookupTransform(
              "dummy_link",
              target_frame_,
              tf2::TimePointZero);
          double dx = t.transform.translation.x;
          double dy = t.transform.translation.y;
          dist = std::hypot(dx, dy);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(),
              1000, "TF lookup (action) failed: %s", ex.what());
        }
      }

      feedback->distance_to_goal = static_cast<float>(dist);
      feedback->arrived = false;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    result->success = false;
    result->message = "Aborted";
    goal_handle->abort(result);
  }

  void Navigate::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
  }

  void Navigate::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pos_[0] = msg->pose.pose.position.x;
    current_pos_[1] = msg->pose.pose.position.y;
  }

  void Navigate::control_cycle()
  {
    if (goal_reached_)
    {
      vel_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }
    if (!last_scan_)
    {
      return;
    }

    VFFVectors vff = get_vff(*last_scan_);

    if (wander_)
    {
      vff.attractive = {1.0f, 0.0f};
    }
    else if (has_goal_ && !goal_reached_)
    {
      try
      {
        auto tf = tf_buffer_.lookupTransform(
            "dummy_link",
            target_frame_,
            tf2::TimePointZero);

        double dx = tf.transform.translation.x;
        double dy = tf.transform.translation.y;
        double dist = std::hypot(dx, dy);

        if (dist <= stop_distance_)
        {
          RCLCPP_INFO(get_logger(), "Reached goal %s", target_frame_.c_str());
          goal_reached_ = true;
          vel_pub_->publish(geometry_msgs::msg::Twist{});
          return;
        }

        double factor = (dist > stop_distance_) ? 1.0 : (dist / stop_distance_) * 0.5;
        vff.attractive = {
            static_cast<float>((dx / dist) * factor),
            static_cast<float>((dy / dist) * factor)};
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "TF lookup failed for %s: %s", target_frame_.c_str(), ex.what());

        vff.attractive = {1.0f, 0.0f};
      }
    }
    else
    {
      vff.attractive = {1.0f, 0.0f};
    }

    auto &r = vff.repulsive;
    float rx = r[0], ry = r[1];
    float rep_mag = std::hypot(rx, ry);
    if (rep_mag > 1e-3f)
    {
      float tx_cw = -ry / rep_mag, ty_cw = rx / rep_mag;
      float tx_ccw = ry / rep_mag, ty_ccw = -rx / rep_mag;

      float ax = vff.attractive[0], ay = vff.attractive[1];
      float score_cw = tx_cw * ax + ty_cw * ay;
      float score_ccw = tx_ccw * ax + ty_ccw * ay;

      float tx = (score_cw > score_ccw ? tx_cw : tx_ccw);
      float ty = (score_cw > score_ccw ? ty_cw : ty_ccw);

      const float tangential_strength = 0.2f;
      r[0] += tx * tangential_strength * rep_mag;
      r[1] += ty * tangential_strength * rep_mag;
    }

    vff.result = {vff.repulsive[0] + vff.attractive[0], vff.repulsive[1] + vff.attractive[1]};

    double angle = std::atan2(vff.result[1], vff.result[0]);
    double speed = std::hypot(vff.result[0], vff.result[1]);

    double signed_speed = speed * std::cos(angle);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(signed_speed, -max_linear_speed_, max_linear_speed_);
    cmd.angular.z = std::clamp(angle, -max_angular_speed_, max_angular_speed_);
    vel_pub_->publish(cmd);

    if (vff_debug_pub_->get_subscription_count() > 0)
    {
      vff_debug_pub_->publish(get_debug_vff(vff));
    }
  }

  double Navigate::calculate_distance_to_goal(double tx, double ty)
  {
    double dx = tx - current_pos_[0];
    double dy = ty - current_pos_[1];
    return std::hypot(dx, dy);
  }

  VFFVectors Navigate::get_vff(const sensor_msgs::msg::LaserScan &scan)
  {

    float influence_distance = this->get_parameter("obstacle_influence_distance").as_double();

    VFFVectors vff_vector{
        {0.0, 0.0}, // attractive
        {0.0, 0.0}, // repulsive
        {0.0, 0.0}  // result
    };

    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
      float range = scan.ranges[i];

      if (!std::isfinite(range) || range > influence_distance)
      {
        continue;
      }

      float angle = scan.angle_min + scan.angle_increment * i;
      float opposite_angle = angle + static_cast<float>(M_PI);

      float force_magnitude = (influence_distance - range) / (range * range);

      force_magnitude = std::min(force_magnitude, 3.0f);

      vff_vector.repulsive[0] += std::cos(opposite_angle) * force_magnitude;
      vff_vector.repulsive[1] += std::sin(opposite_angle) * force_magnitude;
    }

    float repulsive_magnitude = std::sqrt(
        vff_vector.repulsive[0] * vff_vector.repulsive[0] +
        vff_vector.repulsive[1] * vff_vector.repulsive[1]);

    if (repulsive_magnitude > 2.0)
    {
      vff_vector.repulsive[0] = (vff_vector.repulsive[0] / repulsive_magnitude) * 2.0;
      vff_vector.repulsive[1] = (vff_vector.repulsive[1] / repulsive_magnitude) * 2.0;
    }

    return vff_vector;
  }

  visualization_msgs::msg::MarkerArray Navigate::get_debug_vff(const VFFVectors &vff_vectors)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(make_marker(vff_vectors.attractive, BLUE));
    marker_array.markers.push_back(make_marker(vff_vectors.repulsive, RED));
    marker_array.markers.push_back(make_marker(vff_vectors.result, GREEN));
    return marker_array;
  }

  visualization_msgs::msg::Marker Navigate::make_marker(const std::vector<float> &vector, VFFColor color)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->now();
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = vector[0];
    end.y = vector[1];
    end.z = 0.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;

    switch (color)
    {
    case RED:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.id = 0;
      break;
    case GREEN:
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.id = 1;
      break;
    case BLUE:
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.id = 2;
      break;
    }

    marker.color.a = 1.0;
    return marker;
  }

} // namespace robot_nav