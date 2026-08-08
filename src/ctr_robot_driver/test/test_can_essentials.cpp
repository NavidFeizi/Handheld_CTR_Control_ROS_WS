#include <gtest/gtest.h>

#include "ctr_robot_driver/CanEssentials.hpp"

// Pure CiA-402 helpers — compiled against CanEssentials.cpp only (no lely
// master, no CAN hardware).

TEST(CanEssentials, Bin2Dec)
{
  EXPECT_EQ(bin2Dec("0000000000000000"), 0u);
  EXPECT_EQ(bin2Dec("0000000000001111"), 15u);
  EXPECT_EQ(bin2Dec("0001001000110100"), 0x1234u);
  EXPECT_EQ(bin2Dec("1111111111111111"), 65535u);
}

TEST(CanEssentials, StatusWordBits)
{
  StatusWord sw;
  sw.update(0x0027); // ready(0) + switched_on(1) + op_enabled(2) + voltage(5)? bits 0,1,2,5
  EXPECT_TRUE(sw.ready_to_switch_ON);
  EXPECT_TRUE(sw.switched_ON);
  EXPECT_TRUE(sw.operation_enabled);
  EXPECT_FALSE(sw.fault);

  sw.update(0x0008); // fault bit only
  EXPECT_TRUE(sw.fault);
  EXPECT_FALSE(sw.operation_enabled);
}

TEST(CanEssentials, FlagsAtomicSetGet)
{
  Flags flags;
  EXPECT_FALSE(flags.get(Flags::FlagIndex::BOOT_SUCCESS));
  flags.set(Flags::FlagIndex::BOOT_SUCCESS, true);
  flags.set(Flags::FlagIndex::ENABLE_FAULT, true);
  EXPECT_TRUE(flags.get(Flags::FlagIndex::BOOT_SUCCESS));
  EXPECT_TRUE(flags.get(Flags::FlagIndex::ENABLE_FAULT));
  flags.set(Flags::FlagIndex::ENABLE_FAULT, false);
  EXPECT_FALSE(flags.get(Flags::FlagIndex::ENABLE_FAULT));
}

TEST(CanEssentials, ToBinaryStringWidth)
{
  const auto s16 = ToBinaryString<uint16_t>(0x000F);
  // 16 bits + 3 spaces
  EXPECT_EQ(s16.size(), 19u);
  EXPECT_EQ(s16.substr(s16.size() - 4), "1111");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
