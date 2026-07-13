#include "champ_nav2_controller/champ_nav2_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace champ_nav2_controller
{

void ChampController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("ChampController::configure - failed to lock node");
  }

  tf_ = tf;
  plugin_name_ = name;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare our own parameters under the plugin's namespace (e.g.
  // FollowPath.desired_linear_velocity in nav2_params.yaml), same
  // convention every other Nav2 controller plugin uses.
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angle_threshold", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_kp", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".deceleration_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_velocity", rclcpp::ParameterValue(0.05));
  // Seeded from the direct cmd_vel-vs-odom measurement we took by hand:
  // commanding 0.75 rad/s only achieved ~0.31 rad/s actual (~0.41 ratio).
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_angular_scale", rclcpp::ParameterValue(0.41));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".initial_linear_scale", rclcpp::ParameterValue(0.8));

  node->get_parameter(plugin_name_ + ".desired_linear_velocity", desired_linear_velocity_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angle_threshold", rotate_to_heading_angle_threshold_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_kp", rotate_to_heading_kp_);
  node->get_parameter(plugin_name_ + ".deceleration_distance", deceleration_distance_);
  node->get_parameter(plugin_name_ + ".min_linear_velocity", min_linear_velocity_);
  node->get_parameter(plugin_name_ + ".initial_angular_scale", angular_scale_);
  node->get_parameter(plugin_name_ + ".initial_linear_scale", linear_scale_);

  RCLCPP_INFO(
    logger_,
    "ChampController configured: desired_linear=%.2f lookahead=%.2f "
    "initial_angular_scale=%.2f initial_linear_scale=%.2f",
    desired_linear_velocity_, lookahead_distance_, angular_scale_, linear_scale_);
}

void ChampController::cleanup()
{
  RCLCPP_INFO(logger_, "ChampController cleanup");
}

void ChampController::activate()
{
  RCLCPP_INFO(logger_, "ChampController activate");
}

void ChampController::deactivate()
{
  RCLCPP_INFO(logger_, "ChampController deactivate");
  have_last_cmd_ = false;
}

void ChampController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw std::runtime_error("ChampController::setPlan - received an empty path");
  }
  global_plan_ = path;
}

geometry_msgs::msg::PoseStamped ChampController::transformToBaseFrame(
  const geometry_msgs::msg::PoseStamped & pose_in)
{
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    tf_->transform(pose_in, pose_out, "base_link", tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    throw std::runtime_error(
            std::string("ChampController - failed to transform point into base_link: ") +
            ex.what());
  }
  return pose_out;
}

geometry_msgs::msg::PoseStamped ChampController::findLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & robot_pose_in_plan_frame)
{
  const auto & poses = global_plan_.poses;

  // Closest point to the robot = start of the usable path (prunes what's
  // behind us).
  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poses.size(); ++i) {
    const double dx = poses[i].pose.position.x - robot_pose_in_plan_frame.pose.position.x;
    const double dy = poses[i].pose.position.y - robot_pose_in_plan_frame.pose.position.y;
    const double dist = std::hypot(dx, dy);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }

  last_closest_idx_ = closest_idx;

  // Walk forward from the closest point accumulating arc length until we
  // reach lookahead_distance_. If the path ends first, just target the
  // final pose - this naturally makes the robot aim at the goal itself
  // once it's within one lookahead distance of the end.
  double accumulated = 0.0;
  size_t target_idx = closest_idx;
  for (size_t i = closest_idx; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    accumulated += std::hypot(dx, dy);
    target_idx = i + 1;
    if (accumulated >= lookahead_distance_) {
      break;
    }
  }

  return poses[target_idx];
}

double ChampController::remainingPathDistance() const
{
  const auto & poses = global_plan_.poses;
  double remaining = 0.0;
  for (size_t i = last_closest_idx_; i + 1 < poses.size(); ++i) {
    remaining += std::hypot(
      poses[i + 1].pose.position.x - poses[i].pose.position.x,
      poses[i + 1].pose.position.y - poses[i].pose.position.y);
  }
  return remaining;
}

void ChampController::updateCalibration(const geometry_msgs::msg::Twist & actual_velocity)
{
  if (!have_last_cmd_) {
    return;
  }

  if (std::fabs(last_cmd_.angular.z) > kMinCmdForCalibration) {
    double ratio = actual_velocity.angular.z / last_cmd_.angular.z;
    // A sign flip means noise/disturbance dominated that sample, not a
    // real measurement of our own command's effect - skip it rather than
    // let it corrupt the average.
    if (ratio > 0.0) {
      ratio = std::clamp(ratio, kMinScale, kMaxScale);
      angular_scale_ = (1.0 - kCalibrationAlpha) * angular_scale_ + kCalibrationAlpha * ratio;
    }
  }

  if (std::fabs(last_cmd_.linear.x) > kMinCmdForCalibration) {
    double ratio = actual_velocity.linear.x / last_cmd_.linear.x;
    if (ratio > 0.0) {
      ratio = std::clamp(ratio, kMinScale, kMaxScale);
      linear_scale_ = (1.0 - kCalibrationAlpha) * linear_scale_ + kCalibrationAlpha * ratio;
    }
  }
}

geometry_msgs::msg::TwistStamped ChampController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // First: fold this cycle's real, measured velocity into our running
  // calibration estimate, using what we commanded last cycle.
  updateCalibration(velocity);

  if (global_plan_.poses.empty()) {
    throw std::runtime_error("ChampController::computeVelocityCommands - no plan set");
  }

  // The plan lives in its own frame (typically "map"); the robot pose we
  // were handed might be in a different frame (typically "odom"). Bring
  // the robot pose into the plan's frame so distance comparisons in
  // findLookaheadPoint are apples-to-apples.
  geometry_msgs::msg::PoseStamped robot_pose_in_plan_frame = pose;
  if (pose.header.frame_id != global_plan_.header.frame_id) {
    try {
      tf_->transform(
        pose, robot_pose_in_plan_frame, global_plan_.header.frame_id, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      throw std::runtime_error(
              std::string("ChampController - failed to transform robot pose into plan frame: ") +
              ex.what());
    }
  }

  geometry_msgs::msg::PoseStamped lookahead_map = findLookaheadPoint(robot_pose_in_plan_frame);
  lookahead_map.header.stamp = rclcpp::Time(0);  // use latest available transform
  geometry_msgs::msg::PoseStamped lookahead_body = transformToBaseFrame(lookahead_map);

  const double lx = lookahead_body.pose.position.x;
  const double ly = lookahead_body.pose.position.y;
  const double heading_error = std::atan2(ly, lx);

  double desired_linear = 0.0;
  double desired_angular = 0.0;

  if (std::fabs(heading_error) > rotate_to_heading_angle_threshold_) {
    // Too far off-axis to just curve toward it - rotate in place first,
    // like RegulatedPurePursuitController's rotate-to-heading behavior.
    // Proportional control on the heading error, capped at
    // max_angular_velocity_ - this is a REAL, achievable cap now, not the
    // wheeled-robot-style 0.75 rad/s that started this whole investigation.
    desired_angular = std::clamp(
      rotate_to_heading_kp_ * heading_error, -max_angular_velocity_, max_angular_velocity_);
    desired_linear = 0.0;
  } else {
    // Classic pure pursuit curvature: kappa = 2y / L^2, using the real
    // distance to the lookahead point rather than the nominal configured
    // lookahead (matters near the end of the path, where the last pose
    // may be closer than a full lookahead_distance_ away).
    const double L = std::max(std::hypot(lx, ly), 0.05);
    const double curvature = (2.0 * ly) / (L * L);

    // Slow down approaching the end of the path so we don't overshoot the
    // goal the way a constant-speed controller would. Uses distance
    // remaining from the robot's current position (set by
    // findLookaheadPoint above), not the whole original path length.
    const double remaining = remainingPathDistance();
    const double speed_scale = std::clamp(remaining / deceleration_distance_, 0.0, 1.0);
    desired_linear = std::max(
      desired_linear_velocity_ * speed_scale, std::min(min_linear_velocity_, desired_linear_velocity_));
    desired_angular = std::clamp(
      desired_linear * curvature, -max_angular_velocity_, max_angular_velocity_);
  }

  // Apply the live calibration: to actually achieve `desired`, command
  // desired / scale_ (e.g. desired 0.3 rad/s with a learned 0.41 scale ->
  // command 0.73 rad/s, boosting for the gait's real tracking gap).
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = desired_linear / std::max(linear_scale_, kMinScale);
  cmd.angular.z = desired_angular / std::max(angular_scale_, kMinScale);

  last_cmd_ = cmd;
  have_last_cmd_ = true;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist = cmd;
  return cmd_vel;
}

void ChampController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  if (speed_limit < 0) {
    return;
  }
  const double limit = percentage ? (speed_limit / 100.0) * desired_linear_velocity_ : speed_limit;
  desired_linear_velocity_ = std::min(desired_linear_velocity_, limit);
}

}  // namespace champ_nav2_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(champ_nav2_controller::ChampController, nav2_core::Controller)
