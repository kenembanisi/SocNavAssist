#include <gtest/gtest.h>
#include <sarvo_local_planner/utilities.h>
#include <sarvo_local_planner/sarvo_local_planner.h>

using namespace sarvo_local_planner;


TEST(wrapToPi, Correctness)
{
    const double theta1 = M_PI;
    const double theta2 = -M_PI;
    const double theta3 = 2*M_PI;
    const double theta4 = 3*M_PI;
    const double theta5 = -5*M_PI;
    
    EXPECT_EQ(wrapToPi(theta1), M_PI);
    EXPECT_EQ(wrapToPi(theta2), M_PI);
    EXPECT_EQ(wrapToPi(theta3), 0);
    EXPECT_EQ(wrapToPi(theta4), M_PI);
    EXPECT_EQ(wrapToPi(theta5), M_PI);
}


TEST(wrapTo2Pi, Correctness)
{
    const double theta1 = M_PI;
    const double theta2 = -M_PI;
    const double theta3 = 2*M_PI;
    const double theta4 = 3*M_PI;
    const double theta5 = -5*M_PI;
    
    EXPECT_EQ(wrapTo2Pi(theta1), M_PI);
    EXPECT_EQ(wrapTo2Pi(theta2), M_PI);
    EXPECT_EQ(wrapTo2Pi(theta3), 2*M_PI);
    EXPECT_EQ(wrapTo2Pi(theta4), M_PI);
    EXPECT_EQ(wrapTo2Pi(theta5), M_PI);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sarvo_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}