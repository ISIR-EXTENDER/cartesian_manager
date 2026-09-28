#pragma once

#include <optional>
#include <string>
#include <vector>

#include "cartesian_manager/core/shapers/shaper.hpp"

namespace manager_core
{
  struct JointTarget
  {
    std::string name;
    std::vector<double> positions;
  };

  struct JointTargetCommand
  {
    std::string name;
    std::vector<std::string> joint_names;
    std::vector<double> positions;
  };

  struct JointTargetBehaviourConfig
  {
    std::vector<std::string> joint_names;
    std::vector<JointTarget> targets;
  };
} // namespace manager_core
