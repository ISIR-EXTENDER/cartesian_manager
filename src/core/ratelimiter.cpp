#include "cartesian_manager/core/manager.hpp"

#include <memory>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

namespace manager_core
{
  void RateLimiter::reset()
  {
    last_command_ = CartesianVelocity{};
  }

  void RateLimiter::setConfig(const RateLimiterConfig &config)
  {
    config_ = config;
  }

  const RateLimiterConfig &RateLimiter::getConfig() const
  {
    return config_;
  }

  void RateLimiter::update(CartesianVelocity &command, double dt_sec)
  {
    if (dt_sec <= 0.0)
    {
      last_command_ = command;
      return;
    }

    // Linear rate limiter
    if (config_.max_linear_acceleration > 0.0)
    {
      const double max_lin_delta = config_.max_linear_acceleration * dt_sec;
      const Eigen::Vector3d delta_lin = command.linear - last_command_.linear;
      const double delta_lin_norm = delta_lin.norm();
      if (delta_lin_norm > max_lin_delta)
      {
        command.linear = last_command_.linear + (delta_lin / delta_lin_norm) * max_lin_delta;
      }
    }

    // Angular rate limiter
    if (config_.max_angular_acceleration > 0.0)
    {
      const double max_ang_delta = config_.max_angular_acceleration * dt_sec;
      const Eigen::Vector3d delta_ang = command.angular - last_command_.angular;
      const double delta_ang_norm = delta_ang.norm();
      if (delta_ang_norm > max_ang_delta)
      {
        command.angular = last_command_.angular + (delta_ang / delta_ang_norm) * max_ang_delta;
      }
    }

    last_command_ = command;
  }

} // namespace manager_core
