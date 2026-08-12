#include <gtest/gtest.h>

#include "ego_planner/trajectory_diagnostic_binding.h"

namespace ego_planner
{

TEST(TrajectoryDiagnosticBinding, RevisionOnlySchedulesReadOnlyRecheck)
{
  TrajectoryDiagnosticBinding binding;
  EXPECT_FALSE(binding.shouldRecheck(1));

  binding.bind(17, 8, 0.42);
  EXPECT_TRUE(binding.bound());
  EXPECT_EQ(binding.trajectoryId(), 17);
  EXPECT_EQ(binding.publishedMapRevision(), 8u);
  EXPECT_DOUBLE_EQ(binding.minimumClearance(), 0.42);
  EXPECT_FALSE(binding.shouldRecheck(8));
  EXPECT_TRUE(binding.shouldRecheck(9));

  binding.markRechecked(9);
  EXPECT_FALSE(binding.shouldRecheck(9));
  EXPECT_TRUE(binding.shouldRecheck(10));
}

}  // namespace ego_planner
