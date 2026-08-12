#ifndef EGO_PLANNER__TRAJECTORY_DIAGNOSTIC_BINDING_H_
#define EGO_PLANNER__TRAJECTORY_DIAGNOSTIC_BINDING_H_

#include <cstdint>

namespace ego_planner
{

// Read-only bookkeeping for diagnostic logging.  It deliberately contains no
// planning decision, trajectory mutation, or safety-state transition.
class TrajectoryDiagnosticBinding
{
public:
  void bind(int trajectory_id, uint64_t map_revision, double minimum_clearance)
  {
    trajectory_id_ = trajectory_id;
    published_map_revision_ = map_revision;
    last_rechecked_map_revision_ = map_revision;
    minimum_clearance_ = minimum_clearance;
    bound_ = true;
  }

  bool shouldRecheck(uint64_t map_revision) const
  {
    return bound_ && map_revision != last_rechecked_map_revision_;
  }

  void markRechecked(uint64_t map_revision) { last_rechecked_map_revision_ = map_revision; }
  bool bound() const { return bound_; }
  int trajectoryId() const { return trajectory_id_; }
  uint64_t publishedMapRevision() const { return published_map_revision_; }
  double minimumClearance() const { return minimum_clearance_; }

private:
  bool bound_{false};
  int trajectory_id_{-1};
  uint64_t published_map_revision_{0};
  uint64_t last_rechecked_map_revision_{0};
  double minimum_clearance_{0.0};
};

}  // namespace ego_planner

#endif  // EGO_PLANNER__TRAJECTORY_DIAGNOSTIC_BINDING_H_
