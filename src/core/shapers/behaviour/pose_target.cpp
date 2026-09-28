#include "cartesian_manager/core/shapers/behaviour/pose_target.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace manager_core
{
  namespace
  {
    constexpr double kMinQuaternionNorm = 1.0e-9;

    Eigen::Vector3d clampNorm(const Eigen::Vector3d &value, double max_norm)
    {
      if (max_norm <= 0.0)
      {
        return Eigen::Vector3d::Zero();
      }

      const auto norm = value.norm();
      if (norm <= max_norm || norm == 0.0)
      {
        return value;
      }

      return value * (max_norm / norm);
    }

    Eigen::Quaterniond normalized(const Eigen::Quaterniond &quaternion)
    {
      auto result = quaternion;
      result.normalize();
      return result;
    }
  } // namespace

  PoseTarget::PoseTarget(const PoseTargetConfig &config)
  {
    configure(config);
  }

  void PoseTarget::configure(const PoseTargetConfig &config)
  {
    config_ = config;
    pose_targets_.clear();

    const auto target_count = std::min(config_.target_names.size(), config_.targets.size());
    for (std::size_t index = 0; index < target_count; ++index)
    {
      pose_targets_[config_.target_names[index]] = config_.targets[index];
    }

    if (active_ && !dynamic_target_active_ && !hasTarget(active_target_name_))
    {
      reset();
    }
  }

  CartesianCommand PoseTarget::update(const CartesianCommand &, const RobotContext &context,
                                      double)
  {
    CartesianVelocity command;
    const auto target = activeTarget();
    if (!target || validate(context))
    {
      return command;
    }

    const auto linear_error = positionError(*target, context.ee_pose);
    const auto angular_error = orientationError(*target, context.ee_pose);
    command.frame_id = target->frame_id;

    if (linear_error.norm() <= config_.position_tolerance &&
        angular_error.norm() <= config_.orientation_tolerance)
    {
      return command;
    }

    command.linear = clampNorm(config_.linear_kp * linear_error, config_.max_linear_velocity);
    command.angular = clampNorm(config_.angular_kp * angular_error, config_.max_angular_velocity);
    return command;
  }

  void PoseTarget::reset()
  {
    active_ = false;
    dynamic_target_active_ = false;
    dynamic_target_.reset();
    active_target_name_.clear();
  }

  std::string PoseTarget::name() const
  {
    return "pose_target";
  }

  bool PoseTarget::start(const std::string &target_name, const RobotContext &, std::string *error)
  {
    if (!hasTarget(target_name))
    {
      if (error)
      {
        *error = "unknown pose target '" + target_name + "'";
      }
      return false;
    }

    reset();
    active_target_name_ = target_name;
    active_ = true;
    return true;
  }

  bool PoseTarget::start(const CartesianPose &target, std::string *error)
  {
    if (target.frame_id.empty())
    {
      if (error)
      {
        *error = "pose target frame is empty";
      }
      return false;
    }

    if (!target.position.allFinite() || !target.orientation.coeffs().allFinite() ||
        !std::isfinite(target.orientation.norm()) ||
        target.orientation.norm() <= kMinQuaternionNorm)
    {
      if (error)
      {
        *error = "pose target contains a non-finite value or zero quaternion";
      }
      return false;
    }

    auto normalized_target = target;
    normalized_target.orientation.normalize();

    reset();
    dynamic_target_ = std::move(normalized_target);
    dynamic_target_active_ = true;
    active_ = true;
    return true;
  }

  bool PoseTarget::acceptsInputCommand() const
  {
    return false;
  }

  std::optional<std::string> PoseTarget::validate(const RobotContext &context) const
  {
    const auto target = activeTarget();
    if (!target)
    {
      return "no active pose target";
    }

    if (target->frame_id.empty())
    {
      return "active pose target frame is empty";
    }
    if (!target->position.allFinite() || !target->orientation.coeffs().allFinite() ||
        (!std::isfinite(target->orientation.norm()) ||
         target->orientation.norm() <= kMinQuaternionNorm))
    {
      return "active pose target is invalid";
    }
    if (!context.ee_pose.position.allFinite())
    {
      return "current end-effector position is invalid";
    }

    if (context.ee_pose.frame_id.empty())
    {
      return "current end-effector pose frame is empty";
    }

    if (context.ee_pose.frame_id != target->frame_id)
    {
      return "current end-effector pose frame does not match active pose target frame";
    }

    if (!context.ee_pose.orientation.coeffs().allFinite() ||
        (!std::isfinite(context.ee_pose.orientation.norm()) ||
         context.ee_pose.orientation.norm() <= kMinQuaternionNorm))
    {
      return "current end-effector pose orientation is invalid";
    }

    return std::nullopt;
  }

  bool PoseTarget::isComplete(const RobotContext &context) const
  {
    const auto target = activeTarget();
    if (!target || validate(context))
    {
      return false;
    }

    return positionError(*target, context.ee_pose).norm() <= config_.position_tolerance &&
           orientationError(*target, context.ee_pose).norm() <= config_.orientation_tolerance;
  }

  bool PoseTarget::hasTarget(const std::string &name) const
  {
    return pose_targets_.find(name) != pose_targets_.end();
  }

  bool PoseTarget::active() const
  {
    return active_;
  }

  void PoseTarget::addPoseTarget(const std::string &name, const CartesianPose &pose)
  {
    pose_targets_[name] = pose;
  }

  const CartesianPose *PoseTarget::activeTarget() const
  {
    if (dynamic_target_active_)
    {
      return dynamic_target_ ? &*dynamic_target_ : nullptr;
    }

    const auto target = pose_targets_.find(active_target_name_);
    if (target == pose_targets_.end())
    {
      return nullptr;
    }

    return &target->second;
  }

  Eigen::Vector3d PoseTarget::positionError(const CartesianPose &target,
                                            const CartesianPose &current) const
  {
    return target.position - current.position;
  }

  Eigen::Vector3d PoseTarget::orientationError(const CartesianPose &target,
                                               const CartesianPose &current) const
  {
    auto target_orientation = normalized(target.orientation);
    const auto current_orientation = normalized(current.orientation);
    if (target_orientation.dot(current_orientation) < 0.0)
    {
      target_orientation.coeffs() *= -1.0;
    }

    auto error = target_orientation * current_orientation.conjugate();
    error.normalize();

    const Eigen::AngleAxisd angle_axis(error);
    if (!std::isfinite(angle_axis.angle()))
    {
      return Eigen::Vector3d::Zero();
    }

    return angle_axis.axis() * angle_axis.angle();
  }
} // namespace manager_core
