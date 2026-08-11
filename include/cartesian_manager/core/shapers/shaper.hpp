#pragma once

#include <optional>
#include <string>

#include "cartesian_manager/core/types.hpp"

namespace manager_core
{
  class Shaper
  {
  public:
    virtual ~Shaper() = default;

    virtual CartesianCommand update(const CartesianCommand &input, const RobotContext &context,
                                    double dt_sec) = 0;
    virtual void reset() = 0;

    virtual std::string name() const
    {
      return {};
    }
    virtual bool start(const std::string &, const RobotContext &, std::string * = nullptr)
    {
      return false;
    }
    virtual bool acceptsInputCommand() const
    {
      return true;
    }
    virtual std::optional<std::string> validate(const RobotContext &) const
    {
      return std::nullopt;
    }
    virtual bool isComplete(const RobotContext &) const
    {
      return false;
    }
  };

  using Behaviour = Shaper;
} // namespace manager_core
