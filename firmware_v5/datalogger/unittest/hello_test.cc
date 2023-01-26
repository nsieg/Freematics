#include <gtest/gtest.h>
#include "../list.cpp"
#include <iostream>

TEST(BeaconList, ShouldInit) {
  BeaconList bl = BeaconList(5);
  

  Beacon first = bl.get(0);
  Beacon empty = Beacon();

  //std::cout << "Willkommen" << first;

  std::cout << "Willkommen" << bl.size();

  EXPECT_EQ(bl.size(), 0);
  EXPECT_EQ(first, empty);
}