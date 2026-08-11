#include "cartesian_manager/core/shapers/geometric/jaco.hpp"

#include <algorithm>

namespace manager_core
{
  JacoShaper::JacoShaper(const JacoShaperConfig &config) : config_(config)
  {
  }

  CartesianVelocity JacoShaper::update(const CartesianVelocity &input, const RobotContext &context,
                                      double /*dt_sec*/)
  {
    CartesianVelocity output = input;

    const double x_E = context.ee_pose.position.x();
    const double y_E = context.ee_pose.position.y();
    const double v_x = input.linear.x();
    const double v_y = input.linear.y();
    const double denom = x_E * x_E + y_E * y_E;
    const double min_radius = std::max(0.0, config_.min_radius);
    const double min_denom = min_radius * min_radius;

    double omega_z_baseline = 0.0;
    if (denom > min_denom)
    {
      omega_z_baseline = (x_E * v_y - y_E * v_x) / denom;
    }

    if (config_.max_angular_velocity > 0.0)
    {
      omega_z_baseline = std::clamp(omega_z_baseline, -config_.max_angular_velocity,
                                    config_.max_angular_velocity);
    }
    else
    {
      omega_z_baseline = 0.0;
    }

    output.angular = Eigen::Vector3d(0.0, 0.0, omega_z_baseline);
    return output;
  }

  void JacoShaper::reset()
  {
  }
} // namespace manager_core
