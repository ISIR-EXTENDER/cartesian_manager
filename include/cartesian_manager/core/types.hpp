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
    TABLET,
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
    JOINT_TARGET,
    POSE_TARGET
  };

  struct FramesConfig
  {
    std::string ee_frame{"effector_frame"};
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
  };

  struct HybridState
  {
    Eigen::Vector3d previous_hybrid_x_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d previous_angular_input_ = Eigen::Vector3d::Zero();
    double min_cone_ang = 0.09;
    bool inside_cone = false;
    bool has_previous_active_input = false;
    double released_input = 0.01;
  };

  struct RobotContext
  {
    CartesianPose ee_pose;
    CartesianVelocity ee_vel;

    Eigen::MatrixXd ee_jac;

    std::vector<std::string> joint_names;
    Eigen::VectorXd joint_positions;

    HybridState hybrid_state;
    CartesianPose hybrid_frame_pose;

    /**
     * @brief Updates the hybrid orientation frame used for angular input mapping.
     *
     * This implementation is based on the adaptive tool-frame strategy described in:
     * "Intuitive Adaptive Orientation Control for Enhanced Human-Robot Interaction"
     * by Campeau-Lecours et al.
     *
     * The frame follows the tool Z axis while adapting its X/Y axes near vertical
     * singular configurations to preserve intuitive and continuous orientation control.
     *
     * @param angular_input Angular command used to detect release/reversal and update
     *                      the hybrid-frame behavior.
     */
    void updateHybridPose(const Eigen::Vector3d &angular_input);
  };
} // namespace manager_core
