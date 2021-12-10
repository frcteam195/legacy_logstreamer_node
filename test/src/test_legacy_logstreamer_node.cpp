#include "test_legacy_logstream_node.hpp"
#include "legacy_logstreamer_node.hpp"
#include "ros/ros.h"


#include "Timer.hpp"
#include <gtest/gtest.h>
#include <thread>

TEST(LegacyLogstreamerTest, GTestTest)
{
    ASSERT_TRUE(true);
    ASSERT_FALSE(false);
    ASSERT_DOUBLE_EQ(TEST_VAL, 0.00201);
}

TEST(LegacyLogstreamerTest, ElapsedTimerTest)
{
    ck::ElapsedTimer eTimer;
    eTimer.start();
    std::this_thread::sleep_for((std::chrono::milliseconds(1500)));
    double testVal = eTimer.hasElapsed();
    ASSERT_NEAR(testVal, 1.5, 0.1);
}

TEST(LegacyLogstreamerTest, TimeoutTimerTest)
{
    ck::TimeoutTimer timeoutTimer(1.7);
    ASSERT_FALSE(timeoutTimer.isTimedOut());
    std::this_thread::sleep_for((std::chrono::milliseconds(1300)));
    ASSERT_NEAR(timeoutTimer.getTimeLeft(), 0.4, 0.1);
    std::this_thread::sleep_for((std::chrono::milliseconds(500)));
    ASSERT_TRUE(timeoutTimer.isTimedOut());
    ASSERT_DOUBLE_EQ(timeoutTimer.getTimeLeft(), 0);
}

TEST(LegacyLogstreamerTest, IPStrTest)
{
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    inet_aton("10.1.95.2", &servaddr.sin_addr);
    servaddr.sin_port = htons(5809);
    std::string ipTest = sockaddrToIPStr((sockaddr&)servaddr);
    ASSERT_EQ(ipTest, "10.1.95.2");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_legacy_logstreamer_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}