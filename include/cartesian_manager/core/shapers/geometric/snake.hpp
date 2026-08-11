#pragma once

#include "cartesian_manager/core/shapers/shaper.hpp"

namespace manager_core
{
  struct SnakeShaperConfig
  {
    double gain{3.0};
  };

  class SnakeShaper : public Shaper
  {
  public:
    explicit SnakeShaper(const SnakeShaperConfig &config = SnakeShaperConfig{});

    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;

  private:
    SnakeShaperConfig config_;
  };
} // namespace manager_core
