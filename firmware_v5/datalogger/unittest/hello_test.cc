#include <gtest/gtest.h>
#include "../list.h"

// Demonstrate some basic assertions.
TEST(HelloTest, TellTruth) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, truth);
}