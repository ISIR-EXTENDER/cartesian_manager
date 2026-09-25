#pragma once

#include <deque>
#include <string>

#include "cartesian_manager/core/shapers/shaper.hpp"
#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  struct IntentScalingConfig
  {
    double min_scale{0.4};             // linear scale at the start of a push
    double window_sec{0.5};            // how far back the push is measured
    double consistency_threshold{0.5}; // mean push along the current direction above which the scale rises
    double gain{1.0};                  // scale change per second per unit of consistency off threshold
  };

  // The longer the operator pushes the same way, the closer the linear command gets to full scale.
  // Output stays within unit scale, so the downstream max speed remains the ceiling.
  class IntentScaling : public Shaper
  {
  public:
    IntentScaling() = default;
    explicit IntentScaling(const IntentScalingConfig &config);

    void configure(const IntentScalingConfig &config);
    CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                            double dt_sec) override;

    void reset() override;
    std::string name() const override;

    double scale() const;
    double consistency() const;

  private:
    struct Sample
    {
      double dt_sec;
      Eigen::Vector3d displacement;
    };

    IntentScalingConfig config_;
    std::deque<Sample> window_;
    Eigen::Vector3d window_sum_ = Eigen::Vector3d::Zero();
    double window_duration_{0.0};
    double consistency_{0.0};
    double scale_{0.4};
  };

} // namespace manager_core
