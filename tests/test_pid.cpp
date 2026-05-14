#include "pid.h"

#include <gtest/gtest.h>

TEST(PID, StepCommandPullsTowardTarget) {
  PID  pid {1.0, 0.0, 0.0};
  auto out = pid.compute(/*target_x=*/1.0, /*target_y=*/0.0,
                         /*current_x=*/0.0, /*current_y=*/0.0, /*dt=*/0.05);
  EXPECT_GT(out.x, 0.0);
  EXPECT_DOUBLE_EQ(out.y, 0.0);
}

TEST(PID, ZeroErrorYieldsZeroOutputForProportionalOnlyGain) {
  PID  pid {1.0, 0.0, 0.0};
  auto out = pid.compute(0.0, 0.0, 0.0, 0.0, 0.05);
  EXPECT_DOUBLE_EQ(out.x, 0.0);
  EXPECT_DOUBLE_EQ(out.y, 0.0);
}

TEST(PID, IntegralAccumulatesOverRepeatedError) {
  PID  pid {0.0, 1.0, 0.0};
  auto first  = pid.compute(1.0, 0.0, 0.0, 0.0, 0.05);
  auto second = pid.compute(1.0, 0.0, 0.0, 0.0, 0.05);
  // With kp=0 and kd=0, only the integral term contributes; it grows monotonically
  // until it hits the internal clamp.
  EXPECT_GT(second.x, first.x);
}

TEST(PID, NegativeErrorYieldsNegativeProportional) {
  PID  pid {2.0, 0.0, 0.0};
  auto out = pid.compute(/*target_x=*/0.0, /*target_y=*/0.0,
                         /*current_x=*/3.0, /*current_y=*/-1.0, /*dt=*/0.05);
  EXPECT_LT(out.x, 0.0);
  EXPECT_GT(out.y, 0.0);
}
