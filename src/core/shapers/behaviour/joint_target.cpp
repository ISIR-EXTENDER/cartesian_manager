#include "cartesian_manager/core/shapers/behaviour/joint_target.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <regex>
#include <utility>

namespace manager_core
{

  JointTarget::JointTarget(const JointTargetConfig &config)
  {
    configure(config);
  }

  void JointTarget::reset()
  {
    isActive_ = false;
  }

  void JointTarget::configure(const JointTargetConfig &config)
  {
    config_ = config;
    joint_targets_.clear();

    const auto target_count = std::min(config_.target_names.size(), config_.positions.size());
    for (std::size_t index = 0; index < target_count; ++index)
    {
      joint_targets_[config_.target_names[index]] = config_.positions[index];
    }
  }

  void JointTarget::addJointTarget(const std::string &name, const Eigen::VectorXd &joints)
  {
    JointTargetCommand target;
    target.name = name;
    target.positions = joints;
    config_.target_names.push_back(name);
    config_.positions.push_back(target);
  }

  std::string JointTarget::name() const
  {
    return "joint_target";
  }

  std::string JointTarget::activeTargetName() const
  {
    return active_target_.name;
  }

  std::optional<std::string> JointTarget::validate(const RobotContext &context) const
  {
    return std::optional<std::string>();
  }

  bool JointTarget::start(const std::string &target_name, const RobotContext &context,
                          std::string *error)
  {
    return true;
  }

  bool JointTarget::update(const Eigen::VectorXd &joints)
  {
    if (!isActive_ ||
        (active_target_.name != "joint_sliders" && active_target_.name != "wrist_return"))
      return false;

    Eigen::VectorXd robot_positions = joints;

    if (robot_positions.size() != active_target_.positions.size())
    {
      std::cout << "[ERROR] JointTarget::update : Size mismatch : " << robot_positions.size()
                << " != " << active_target_.positions.size() << std::endl;
      return false;
    }
    std::cout << active_target_.name << std::endl;
    active_target_.positions = robot_positions; // copie
    if (active_target_.name == "wrist_return")
    {
      if (robot_positions[3] < -(M_PI / 2))
      {
        if (robot_positions[5] < -(M_PI / 2))
        {
          active_target_.positions[3] += M_PI;
          active_target_.positions[5] += M_PI;
          active_target_.positions[4] *= -1.0;
        }
        if (robot_positions[5] > (M_PI / 2))
        {
          active_target_.positions[3] += M_PI;
          active_target_.positions[5] -= M_PI;
          active_target_.positions[4] *= -1.0;
        }
      }
      if (robot_positions[3] > (M_PI / 2))
      {
        if (robot_positions[5] < -(M_PI / 2))
        {
          active_target_.positions[3] -= M_PI;
          active_target_.positions[5] += M_PI;
          active_target_.positions[4] *= -1.0;
        }
        if (robot_positions[5] > (M_PI / 2))
        {
          active_target_.positions[3] -= M_PI;
          active_target_.positions[5] -= M_PI;
          active_target_.positions[4] *= -1.0;
        }
      }
    }

    return true;
  }

  bool JointTarget::isComplete(const RobotContext &context) const
  {
    return !isActive_;
  }

  bool JointTarget::hasTarget(const std::string &name) const
  {
    return joint_targets_.find(name) !=
           joint_targets_.end(); // True si name est dans joint_targets_, faux sinon
  }

  bool JointTarget::isActive() const
  {
    return isActive_;
  }

  bool JointTarget::setActiveTarget(const std::string &name)
  {
    if (!hasTarget(name))
    {
      return false;
    }

    active_target_ = joint_targets_[name];
    isActive_ = true;
    return true;
  }

  bool JointTarget::setUnactive()
  {
    if (!isActive_)
      return false;

    isActive_ = false;
    return true;
  }

  CartesianCommand JointTarget::update(const CartesianCommand &input, const RobotContext &context,
                                       double dt_sec)
  {
    return CartesianCommand();
  }

  const JointTargetCommand *JointTarget::activeTarget() const
  {
    // TODO : verifier si isActive et si hasTarget(active_target_name)
    const auto it = joint_targets_.find(active_target_.name);
    if (it == joint_targets_.end())
    {
      return nullptr;
    }

    return &it->second; //
  }

  JointVelocityCommand JointTarget::getJointVelocityCommand(const RobotContext &context)
  {
    if (context.joint_names.size() != static_cast<std::size_t>(context.joint_positions.size()))
    {
      std::cout << "[ERROR] Joint names and positions size mismatch : "
                << context.joint_names.size() << " != " << context.joint_positions.size()
                << std::endl;
      return JointVelocityCommand{};
    }

    static const std::regex joint_name_pattern("joint_");
    Eigen::VectorXd robot_positions(context.joint_positions.size());
    Eigen::Index joint_position_count = 0;
    for (std::size_t i = 0; i < context.joint_names.size(); ++i)
    {
      if (std::regex_search(context.joint_names[i], joint_name_pattern))
      {
        robot_positions[joint_position_count++] =
            context.joint_positions[static_cast<Eigen::Index>(i)];
      }
    }
    robot_positions.conservativeResize(joint_position_count);

    Eigen::VectorXd target = active_target_.positions;
    JointVelocityCommand velocity_command;

    if (!isActive())
    {
      std::cout << "[ERROR] Target not active ! " << std::endl;
      return velocity_command;
    }

    if (target.size() != robot_positions.size())
    {
      std::cout << "[ERROR] Size mismatch : " << target.size() << " != " << robot_positions.size()
                << std::endl;
      return velocity_command;
    }

    Eigen::VectorXd target_velocity = Eigen::VectorXd::Zero(target.size());
    velocity_command.joint_speed = target_velocity; // Set zero
    Eigen::VectorXd error(robot_positions.size());

    for (Eigen::Index i = 0; i < target.size(); ++i)
    {
      error[i] = std::fmod(target[i] - robot_positions[i], 2.0 * M_PI);
    }

    if (error.lpNorm<Eigen::Infinity>() <= config_.position_tolerance)
    { // Calcule la norme infinie de error et compare avec la tolerance
      isActive_ = false;
      return velocity_command;
      std::cout << "Joint target reached; returning to Cartesian QP control." << std::endl;
    }
    else
    {
      target_velocity = config_.velocity_gain * error;
      const double max_velocity = std::max(0.0, config_.max_velocity);
      if (max_velocity == 0.0)
      {
        target_velocity.setZero();
      }
      else
      {
        const double target_velocity_norm = target_velocity.lpNorm<2>();
        if (target_velocity_norm > max_velocity)
        {
          target_velocity *= max_velocity / target_velocity_norm;
        }
      }
    }

    velocity_command.joint_names = config_.joint_names; // Joint names par defaut [q1 -> q6-7]
    velocity_command.joint_speed = target_velocity;

    return velocity_command;
  }

} // namespace manager_core