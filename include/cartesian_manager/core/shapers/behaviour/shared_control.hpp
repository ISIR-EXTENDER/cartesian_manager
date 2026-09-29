#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "cartesian_manager/core/shapers/shaper.hpp"
#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct SharedControlGoal
  {
    std::string id;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    double confidence{0.0};
  };

  // The law is written in SI (m, rad, m/s, rad/s); context.command_scale converts to and from unit scale.
  struct SharedControlConfig
  {
    double alpha_conf{1.5};              // confidence integration gain [1/s]
    double theta_l{15.0 * M_PI / 180.0}; // confidence cone half-angle [rad]
    double v_j_max{0.055};               // speed at which confidence integrates at full rate [m/s]
    double gamma{2.0};                   // gain on the goal-aligned component
    double r1{0.04};                     // distance gate radii [m]
    double r2{0.02};
    double theta1{15.0 * M_PI / 180.0}; // rotation gate angles [rad]
    double theta2{5.0 * M_PI / 180.0};
    double goal_match_distance{0.05};     // a new goal this close to a previous one keeps its confidence [m]
    double input_scale{1.0};              // headroom for gamma under the downstream cap; 1/gamma is the FR3 ratio
    std::vector<SharedControlGoal> goals; // static goals in the base frame
  };

  struct SharedControlState
  {
    std::vector<std::string> goal_ids; // "agnostic" first, the current end-effector pose
    std::vector<double> confidences;
    CartesianPose soft_goal;
  };

  // Blends the operator's twist with assistance towards the goal they seem to aim at.
  // A linear command drives translation mode and updates confidences; an angular-only command drives
  // rotation mode. Confidences outlive mode changes: only reset() clears them.
  class SharedControl : public Shaper
  {
  public:
    SharedControl() = default;
    explicit SharedControl(const SharedControlConfig &config);

    void configure(const SharedControlConfig &config);
    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;
    std::string name() const override;

    // Replaces the dynamic goals. Each keeps the confidence of the nearest previous goal within
    // goal_match_distance, so a goal re-detected slightly elsewhere is not forgotten.
    void setGoals(const std::vector<SharedControlGoal> &goals);
    std::vector<SharedControlGoal> goals() const;
    SharedControlState state(const RobotContext &context) const;

  private:
    struct Twist
    {
      Eigen::Vector3d linear;
      Eigen::Vector3d angular;
    };

    void updateConfidences(const Eigen::Vector3d &velocity, const Eigen::Vector3d &position,
                           double dt_sec);
    double agnosticConfidence() const;
    CartesianPose softGoal(const CartesianPose &ee_pose) const;
    double sigmaD(double distance) const;
    double sigmaR(double angle) const;
    Twist translationMode(const Twist &input, const CartesianPose &goal,
                          const CartesianPose &ee_pose) const;
    Twist rotationMode(const Twist &input, const CartesianPose &goal,
                       const CartesianPose &ee_pose) const;

    template <typename Function> void forEachGoal(Function function);
    template <typename Function> void forEachGoal(Function function) const;

    SharedControlConfig config_;
    std::vector<SharedControlGoal> static_goals_;
    std::vector<SharedControlGoal> dynamic_goals_;
  };

} // namespace manager_core
