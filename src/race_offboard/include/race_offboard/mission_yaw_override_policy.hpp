#pragma once

#include <cmath>

namespace race_offboard
{
inline bool missionYawOverrideAllowed(
  bool enabled, bool have_command, bool armed, bool offboard, bool active,
  bool goal_reached_hold, double command_age_sec, double timeout_sec)
{
  return enabled && have_command && armed && offboard && active && goal_reached_hold &&
         std::isfinite(command_age_sec) && command_age_sec >= 0.0 &&
         std::isfinite(timeout_sec) && timeout_sec > 0.0 && command_age_sec <= timeout_sec;
}
}  // namespace race_offboard
