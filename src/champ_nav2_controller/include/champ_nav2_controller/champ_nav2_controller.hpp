#ifndef CHAMP_NAV2_CONTROLLER__CHAMP_NAV2_CONTROLLER_HPP_
#define CHAMP_NAV2_CONTROLLER__CHAMP_NAV2_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace champ_nav2_controller
{

class ChampController : public nav2_core::Controller
{
public:
  ChampController() = default;
  ~ChampController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // Finds the point on global_plan_ at ~lookahead_distance_ ahead of the
  // robot, expressed in the plan's own frame (map). Also acts as our path
  // pruning step - anything behind the closest point is ignored.
  geometry_msgs::msg::PoseStamped findLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & robot_pose_in_plan_frame);

  // Transforms a pose into the robot's base frame using tf2, so its (x, y)
  // become "forward" / "left-right" relative to the robot - what the pure
  // pursuit curvature formula needs.
  geometry_msgs::msg::PoseStamped transformToBaseFrame(
    const geometry_msgs::msg::PoseStamped & pose_in);

  // Compares the command we sent LAST cycle against the velocity Nav2 says
  // we actually achieved THIS cycle, and folds that ratio into a running
  // exponential-moving-average correction factor.
  void updateCalibration(const geometry_msgs::msg::Twist & actual_velocity);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("ChampController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;

  // --- parameters (declared in configure(), see .cpp for defaults/meaning) ---
  double desired_linear_velocity_;
  double lookahead_distance_;
  double max_angular_velocity_;
  double rotate_to_heading_angle_threshold_;
  double rotate_to_heading_kp_;
  double deceleration_distance_;
  double min_linear_velocity_;

  double speed_limit_ = -1.0;  // -1 = no limit applied
  bool speed_limit_is_percentage_ = false;

  // --- live calibration state ---
  // "scale_" is how much of a commanded velocity actually shows up in real
  // motion (e.g. 0.4 means the robot only achieves 40% of what we ask for).
  // We seed it from a measured value rather than assuming 1.0, then let it
  // adapt. To realize a desired real-world velocity, we command
  // desired / scale_.
  double angular_scale_;
  double linear_scale_;
  bool have_last_cmd_ = false;
  geometry_msgs::msg::Twist last_cmd_;

  static constexpr double kCalibrationAlpha = 0.1;
  static constexpr double kMinCmdForCalibration = 0.05;
  static constexpr double kMinScale = 0.15;
  static constexpr double kMaxScale = 2.0;
};

}  // namespace champ_nav2_controller

#endif  // CHAMP_NAV2_CONTROLLER__CHAMP_NAV2_CONTROLLER_HPP_
