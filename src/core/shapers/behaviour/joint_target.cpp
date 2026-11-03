#include "cartesian_manager/core/shapers/behaviour/joint_target.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace manager_core
{

  JointTarget::JointTarget(const JointTargetConfig &config)
  {
    configure(config);
  }

  // JointTarget::JointTarget(const JointVelocityCommand &velocity_command)
  // {
  //     velocity_command_ = velocity_command;
  // }

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

  std::optional<std::string> JointTarget::validate(const RobotContext &context) const
  {
    return std::optional<std::string>();
  }

  void JointTarget::reset()
  {
      isActive_ = false;
      // dynamic_target_active_ = false;
      // dynamic_target_.reset();
      // active_target_name_.clear();
  }

  CartesianCommand JointTarget::update(const CartesianCommand &input, const RobotContext &context,
                                       double dt_sec)
  {
    return CartesianCommand();
  }

  std::string JointTarget::name() const
  {
    return "joint_target";
  }

  bool JointTarget::start(const std::string &target_name, const RobotContext &context,
             std::string *error)
  {
    return true;
  }

  /*bool JointTarget::start(const Eigen::VectorXd &target, std::string *error = nullptr)
  {
    return true;
  }*/

  std::optional<std::string> validate(const RobotContext &context)
  {
    return std::nullopt;
  }
  bool isComplete(const RobotContext &context)
  {
    return true;
  }

  bool JointTarget::isComplete(const RobotContext &context) const
  {
    return false;
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

  std::string JointTarget::activeTargetName() const
  {
    return active_target_.name;
  }

  void addJointTarget(const std::string &name, const Eigen::VectorXd &joints)
  {
    ;
  }

  bool JointTarget::setActiveTarget(const std::string &name)
  {
    if(!hasTarget(name)){
      return false;
    }

    active_target_ = joint_targets_[name];
    return true;

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

  JointVelocityCommand JointTarget::getJointVelocityCommand(const RobotContext &context) const
  {
    Eigen::VectorXd robot_position = context.joint_positions;
    Eigen::VectorXd qd; // TODO : a definir
    JointVelocityCommand velocity_command;

    if (!isActive())
    {
      return velocity_command;
    }

    if (qd.size() != robot_position.size())
    {
      return velocity_command;
    }

    Eigen::VectorXd target_velocity = Eigen::VectorXd::Zero(qd.size());
    Eigen::VectorXd error(robot_position.size());

    for (Eigen::Index i = 0; i < qd.size(); ++i)
    {
      error[i] = std::remainder(robot_position[i] - qd[i], 2.0 * M_PI);
    }

    if (error.lpNorm<Eigen::Infinity>() <= config_.position_tolerance)
    { // Calcule la norme infinie de error et compare avec la tolerance
      //isActive_ = false;
      //"Joint target reached; returning to Cartesian QP control."
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
        for (Eigen::Index i = 0; i < target_velocity.size(); ++i)
        {
          target_velocity[i] = std::max(
              -max_velocity,
              std::min(max_velocity, target_velocity[i])); // Contraint la vitesse ntre -max et +max
        }
      }
    }

    velocity_command.joint_names = config_.joint_names; // Joint names par defaut [q1 -> q6-7]
    velocity_command.joint_speed = target_velocity;
    return velocity_command;
  }

} // namespace manager_core