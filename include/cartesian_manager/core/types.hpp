#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace manager_core
{

  enum class InputSource
  {
    JOYSTICK,
    VISUAL_SERVOING
  };

  enum class Geometrics
  {
    BOTH,
    JACO,
    SNAKE,
  };

  enum class Behaviours
  {
    PASSTHROUGH,
    JOINT_TARGET
  };

  struct FramesConfig
  {
    std::string ee_frame{"ft_frame"};
    std::string base_frame{"base_link"};
    std::string hybrid_frame{"hybrid_frame"};
  };

  struct CartesianPose
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::string frame_id;
  };

  struct CartesianVelocity
  {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular = Eigen::Vector3d::Zero();
    std::string frame_id;
  };

  using CartesianCommand = CartesianVelocity;

  struct TimedCartesianCommand
  {
    CartesianCommand command;
    double stamp_sec = 0.0;
    bool received = false;
  };

  struct InputChannel
  {
    TimedCartesianCommand latest;
    double timeout{0.2};
    bool enabled{true};
    double weight{1.};
  };

  struct RobotContext
  {
    CartesianPose ee_pose;
    CartesianPose hybrid_pose;
    CartesianVelocity ee_vel;
    Eigen::MatrixXd ee_jac;
    std::vector<std::string> joint_names;
    Eigen::VectorXd joint_positions;
  };
} // namespace manager_core
