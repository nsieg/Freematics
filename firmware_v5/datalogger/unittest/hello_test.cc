#include <gtest/gtest.h>
#include "../list.cpp"
#include <iostream>

TEST(BeaconList, ShouldThrowOnWrongIndex) {
  BeaconList bl = BeaconList(5);

  EXPECT_THROW(bl.get(0), std::out_of_range);
  EXPECT_THROW(bl.get(1), std::out_of_range);
  EXPECT_THROW(bl.get(5), std::out_of_range);
  EXPECT_THROW(bl.get(6), std::out_of_range);
}

TEST(BeaconList, ShouldReturnInitialized) {
  BeaconList bl = BeaconList(5);
  Beacon b = Beacon("1A:2B", -50);
  bl.add(b);

  EXPECT_EQ(bl.get(0), b);
}

TEST(BeaconList, ShouldReturnSorted) {
  BeaconList bl = BeaconList(5);
  BeaconList bl2 = BeaconList(5);
  Beacon far = Beacon("1A:2B", -50);
  Beacon near = Beacon("1A:2B:3C", -10);
  bl.add(far);
  bl.add(near);
  bl2.add(near);
  bl2.add(far);

  EXPECT_EQ(bl.get(0), far);
  EXPECT_EQ(bl.get(1), near);
  EXPECT_EQ(bl2.get(0), far);
  EXPECT_EQ(bl2.get(1), near);
}

TEST(BeaconList, ShouldNotAddDuplicate) {
  BeaconList bl = BeaconList(5);
  Beacon one = Beacon("1A:2B", -50);
  Beacon two = Beacon("1A:2B", -50);
  bl.add(one);
  bl.add(two);

  EXPECT_EQ(bl.size(), 1);
}

TEST(BeaconList, ShouldUpdate) {
  BeaconList bl = BeaconList(5);
  Beacon one = Beacon("1A:2B", -50);
  Beacon two = Beacon("1A:2B", -60);
  bl.add(one);
  bl.add(two);

  EXPECT_EQ(bl.size(), 1);
  EXPECT_EQ(bl.get(0), two);
}

TEST(BeaconList, ShouldPrintCsv) {
  BeaconList bl = BeaconList(5);
  Beacon one = Beacon("1A:2B:3C", -50);
  Beacon two = Beacon("1A:2B:4F", -30);
  Beacon three = Beacon("1A:2B:66", -60);
  bl.add(one);
  bl.add(two);
  bl.add(three);

  string expected = "1A:2B:4F,-30,1A:2B:3C,-50,1A:2B:66,-60";
  EXPECT_EQ(bl.toCsvString(), expected);
}

TEST(BeaconList, ShouldNotOverGrow) {
  BeaconList bl = BeaconList(3);
  Beacon one = Beacon("1A:2B:3C", -50);
  Beacon two = Beacon("1A:2B:4F", -30);
  Beacon three = Beacon("1A:2B:66", -60);
  Beacon four = Beacon("1A:2B:66:99", -40);
  bl.add(one);
  bl.add(two);
  bl.add(three);
  bl.add(four);

  EXPECT_EQ(bl.size(), 3);
  EXPECT_EQ(bl.get(0), one);
  EXPECT_EQ(bl.get(1), four);
  EXPECT_EQ(bl.get(2), two);
}
