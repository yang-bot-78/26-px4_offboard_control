#pragma once

#include <cmath>
#include <cstdint>

namespace race_offboard
{

// VehicleLocalPosition carries delta_z only for its most recent reset.  The
// counter therefore makes the delta an edge-triggered event instead of a
// persistent value that could be applied once per received PX4 sample.
enum class Px4LocalZResetDecision
{
  Initialized,
  Duplicate,
  Applied,
  RejectedNonFinite,
};

struct Px4LocalZResetObservation
{
  Px4LocalZResetDecision decision{Px4LocalZResetDecision::Initialized};
  double delta_z_ned{0.0};
  double cumulative_delta_z_ned{0.0};
};

class Px4LocalZResetTracker
{
public:
  Px4LocalZResetObservation observe(std::uint8_t reset_counter, double delta_z_ned)
  {
    if (!initialized_) {
      initialized_ = true;
      reset_counter_ = reset_counter;
      return {Px4LocalZResetDecision::Initialized, 0.0, cumulative_delta_z_ned_};
    }

    if (reset_counter == reset_counter_) {
      return {Px4LocalZResetDecision::Duplicate, 0.0, cumulative_delta_z_ned_};
    }

    if (!std::isfinite(delta_z_ned)) {
      // Do not advance the counter. A later sample with the same reset counter
      // can still carry the valid delta and must remain actionable.
      return {Px4LocalZResetDecision::RejectedNonFinite, 0.0, cumulative_delta_z_ned_};
    }

    reset_counter_ = reset_counter;
    cumulative_delta_z_ned_ += delta_z_ned;
    return {Px4LocalZResetDecision::Applied, delta_z_ned, cumulative_delta_z_ned_};
  }

  double cumulativeDeltaZNed() const
  {
    return cumulative_delta_z_ned_;
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  std::uint8_t reset_counter_{0};
  double cumulative_delta_z_ned_{0.0};
  bool initialized_{false};
};

// A cached or unaligned incoming target is expressed in PX4 local NED.
inline double rebaseLocalNedZ(double z_ned, double delta_z_ned)
{
  return z_ned + delta_z_ned;
}

// PlanarFrameTransform::z is local ENU, whose positive direction is opposite
// to PX4 local NED Z.
inline double rebaseMapToLocalEnuZ(double map_to_local_z_enu, double delta_z_ned)
{
  return map_to_local_z_enu - delta_z_ned;
}

}  // namespace race_offboard
