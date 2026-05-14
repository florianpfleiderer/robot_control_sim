#include "robot.h"

#include <cmath>
#include <gtest/gtest.h>

TEST(Robot, ConstructsAtOrigin) {
  Robot r;
  const auto [x, y]   = r.position();
  const auto [vx, vy] = r.velocity();
  EXPECT_DOUBLE_EQ(x, 0.0);
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(vx, 0.0);
  EXPECT_DOUBLE_EQ(vy, 0.0);
}

TEST(Robot, ConstantDisturbanceDriftsWithZeroCommand) {
  // The Robot model includes a constant disturbance (+x, -y) so a robot
  // with no command should drift in those directions.
  Robot r;
  for (int i = 0; i < 50; ++i) {
    r.update(0.0, 0.0, 0.05);
  }
  const auto [x, y] = r.position();
  EXPECT_GT(x, 0.0);
  EXPECT_LT(y, 0.0);
}

TEST(Robot, PositiveCommandAcceleratesFurtherThanDisturbanceAlone) {
  Robot disturbed;
  Robot commanded;
  for (int i = 0; i < 50; ++i) {
    disturbed.update(0.0, 0.0, 0.05);
    commanded.update(10.0, 0.0, 0.05);
  }
  EXPECT_GT(std::get<0>(commanded.position()),
            std::get<0>(disturbed.position()));
}

TEST(Robot, FrictionBoundsVelocityUnderConstantCommand) {
  // With friction 0.98 per step, a constant accel reaches a finite terminal
  // velocity rather than diverging.
  Robot r;
  for (int i = 0; i < 1000; ++i) {
    r.update(1.0, 0.0, 0.05);
  }
  const auto [vx, vy] = r.velocity();
  EXPECT_TRUE(std::isfinite(vx));
  EXPECT_TRUE(std::isfinite(vy));
  EXPECT_LT(std::abs(vx), 100.0);
  EXPECT_LT(std::abs(vy), 100.0);
}
