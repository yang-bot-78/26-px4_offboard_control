#pragma once

#include <algorithm>
#include <cmath>

namespace race_super_planner_ros2
{

struct GlobalGridGeometry
{
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.0};
  int width{0};
  int height{0};

  int cellX(double x) const
  {
    return static_cast<int>(std::floor((x - origin_x) / resolution));
  }

  int cellY(double y) const
  {
    return static_cast<int>(std::floor((y - origin_y) / resolution));
  }

  bool contains(double x, double y) const
  {
    const int cell_x = cellX(x);
    const int cell_y = cellY(y);
    return cell_x >= 0 && cell_x < width && cell_y >= 0 && cell_y < height;
  }
};

inline GlobalGridGeometry makeSharedBoundsGrid(
  double min_x, double max_x, double min_y, double max_y, double resolution)
{
  GlobalGridGeometry grid;
  grid.resolution = resolution;
  grid.origin_x = std::floor(min_x / resolution) * resolution;
  grid.origin_y = std::floor(min_y / resolution) * resolution;
  // The maximum boundary itself must map to a valid cell.  A ceil-only size
  // would exclude it whenever max lies exactly on a cell boundary.
  grid.width = std::max(
    1, static_cast<int>(std::floor(
      (max_x - grid.origin_x) / resolution + 1.0e-9)) + 1);
  grid.height = std::max(
    1, static_cast<int>(std::floor(
      (max_y - grid.origin_y) / resolution + 1.0e-9)) + 1);
  return grid;
}

inline bool insideSharedBounds(
  double x, double y, double min_x, double max_x, double min_y, double max_y)
{
  return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
}

// makeSharedBoundsGrid deliberately includes the cell containing the maximum
// boundary, so a goal exactly on that boundary has a valid grid index.  The
// centre of that extra cell can nevertheless lie beyond the centre-safe
// geofence.  A* may use only cells whose *centres* remain in the bounds;
// endpoint connectors are validated separately against the real endpoint.
inline bool cellCenterInsideSharedBounds(
  int cell_x, int cell_y, const GlobalGridGeometry & grid,
  double min_x, double max_x, double min_y, double max_y)
{
  const double x = grid.origin_x + (static_cast<double>(cell_x) + 0.5) * grid.resolution;
  const double y = grid.origin_y + (static_cast<double>(cell_y) + 0.5) * grid.resolution;
  return insideSharedBounds(x, y, min_x, max_x, min_y, max_y);
}

struct PlanningGoalSelection
{
  double x{0.0};
  double y{0.0};
  bool clipped{false};
};

inline PlanningGoalSelection selectPlanningGoal(
  bool global_only, double start_x, double start_y, double final_x, double final_y,
  double local_distance_limit)
{
  PlanningGoalSelection selection{final_x, final_y, false};
  if (global_only) {
    return selection;
  }
  const double dx = final_x - start_x;
  const double dy = final_y - start_y;
  const double distance = std::hypot(dx, dy);
  if (distance > local_distance_limit) {
    selection.x = start_x + dx / distance * local_distance_limit;
    selection.y = start_y + dy / distance * local_distance_limit;
    selection.clipped = true;
  }
  return selection;
}

}  // namespace race_super_planner_ros2
