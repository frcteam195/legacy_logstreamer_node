#include "ros/ros.h"

#include "Timer.hpp"
#include <gtest/gtest.h>

TEST(LegacyLogstreamerTest, GTestTest)
{
    ASSERT_TRUE(true);
    ASSERT_FALSE(false);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_legacy_logstreamer_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}