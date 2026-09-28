#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "cartesian_manager/core/shapers/shaper.hpp"
#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct PoseTargetConfig
  {
    double linear_kp{1.0};
    double angular_kp{1.0};
    double max_linear_velocity{0.1};
    double max_angular_velocity{0.2};
    double position_tolerance{0.05};
    double orientation_tolerance{0.05};

    std::vector<std::string> target_names;
    std::vector<CartesianPose> targets;
  };

  class PoseTarget : public Shaper
  {
  public:
    PoseTarget() = default;
    explicit PoseTarget(const PoseTargetConfig &config);

    void configure(const PoseTargetConfig &config);
    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;
    std::string name() const override;
    bool start(const std::string &target_name, const RobotContext &context,
               std::string *error = nullptr) override;
    bool start(const CartesianPose &target, std::string *error = nullptr);
    bool acceptsInputCommand() const override;
    std::optional<std::string> validate(const RobotContext &context) const override;
    bool isComplete(const RobotContext &context) const override;

    bool hasTarget(const std::string &name) const;
    bool active() const;
    void addPoseTarget(const std::string &name, const CartesianPose &pose);

  private:
    const CartesianPose *activeTarget() const;
    Eigen::Vector3d positionError(const CartesianPose &target, const CartesianPose &current) const;
    Eigen::Vector3d orientationError(const CartesianPose &target,
                                     const CartesianPose &current) const;

    PoseTargetConfig config_;
    std::unordered_map<std::string, CartesianPose> pose_targets_;
    std::optional<CartesianPose> dynamic_target_;
    std::string active_target_name_;
    bool dynamic_target_active_{false};
    bool active_{false};
  };

} // namespace manager_core
