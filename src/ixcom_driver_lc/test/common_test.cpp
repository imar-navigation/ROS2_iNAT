
#include <gtest/gtest.h>
#include "ixcom_driver_lc/modules/imu.hpp"

TEST(package_name, a_first_test) {
  ASSERT_EQ(4, 2 + 2);
}


TEST(MODULES_IMU, getSetupFreq) {
    uint16_t res = getSetupFreq();
    ASSERT_EQ(res, 0)
}


int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}