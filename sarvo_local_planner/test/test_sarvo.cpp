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
    // EXPECT_EQ(wrapTo2Pi(theta3), 2*M_PI);
    EXPECT_EQ(wrapTo2Pi(theta4), M_PI);
    EXPECT_EQ(wrapTo2Pi(theta5), M_PI);
}


TEST(angleBetween, Correctness)
{
    Point2D point1; point1.x = 0.0; point1.y = 1.0; // [-1, 0]
    Point2D point2; point2.x = 1.0; point2.y = 0.0; // [0, 1]
    Point2D point3; point3.x = 0.0; point3.y = -1.0; // [1, 0]
    Point2D point4; point4.x = 0.5; point4.y = 0.866; // [-0.866, 0.5]
    Point2D point5; point5.x = -0.5; point5.y = -1; // [1, -0.5]
    Point2D point6; point6.x = -1.0; point6.y = 0.0; // [0, -1]
    Point2D point7; point7.x = 0.866; point7.y = -0.5; // [0.5, 0.866]
    
    EXPECT_DOUBLE_EQ(angleBetween(point1, point2), PI/2);
    EXPECT_DOUBLE_EQ(angleBetween(point2, point1), -PI/2);
    EXPECT_DOUBLE_EQ(angleBetween(point1, point6), -PI/2);
    EXPECT_DOUBLE_EQ(angleBetween(point1, point3), PI);
    EXPECT_DOUBLE_EQ(angleBetween(point3, point4), -2.6179811758198239);
    EXPECT_DOUBLE_EQ(angleBetween(point3, point5), 0.46364760900080609);
    EXPECT_DOUBLE_EQ(angleBetween(point4, point5), 3.0816287848206301);
    EXPECT_DOUBLE_EQ(angleBetween(point1, point5), -2.677945044588987);
    EXPECT_DOUBLE_EQ(angleBetween(point2, point5), 2.0344439357957027);
    EXPECT_DOUBLE_EQ(angleBetween(point1, point4), 0.52361147776996919);
    EXPECT_DOUBLE_EQ(angleBetween(point6, point5), -1.107148717794091);
    EXPECT_DOUBLE_EQ(angleBetween(point2, point7), 0.523611477769969);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sarvo_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}