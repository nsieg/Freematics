#include <gtest/gtest.h>
#include "../list.cpp"

TEST(BeaconList, ShouldInit) {
  BeaconList bl = BeaconList(5);

  EXPECT_EQ(bl.size(), 0);
  //ASSERT_EQ(bl.get(0), Beacon());
}