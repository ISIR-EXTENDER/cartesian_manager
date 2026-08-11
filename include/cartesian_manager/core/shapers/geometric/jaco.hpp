#pragma once

#include "cartesian_manager/core/shapers/shaper.hpp"

namespace manager_core
{
  struct JacoShaperConfig
  {
    double min_radius{0.05};
    double max_angular_velocity{0.4};
  };

  class JacoShaper : public Shaper
  {
  public:
    explicit JacoShaper(const JacoShaperConfig &config = JacoShaperConfig{});

    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;

  private:
    JacoShaperConfig config_;
  };
} // namespace manager_core
