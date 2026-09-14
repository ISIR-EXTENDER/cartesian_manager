#pragma once

#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct RateLimiterConfig
  {
    double max_linear_acceleration{2.0};   // max change per second in normalized linear command (<= 0.0 to disable)
    double max_angular_acceleration{2.0};  // max change per second in normalized angular command (<= 0.0 to disable)
  };

  class RateLimiter
  {
  public:
    explicit RateLimiter(const RateLimiterConfig& config = RateLimiterConfig{});

    /// Apply rate limiting to the command. Returns the limited command.
    void update(CartesianVelocity& command, double dt_sec);

    /// Reset internal state (e.g., when switching modes or on input timeout).
    void reset();

    /// Update configuration at runtime.
    void setConfig(const RateLimiterConfig& config);
    const RateLimiterConfig& getConfig() const;

  private:
    RateLimiterConfig config_;
    CartesianVelocity last_command_{};
  };
} // namespace manager_core