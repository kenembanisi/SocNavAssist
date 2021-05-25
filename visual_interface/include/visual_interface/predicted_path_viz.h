// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
// #include "visual_interface/robotState.h"
#include "visual_interface/trajectory.h"
#include "visual_interface/trajectoryPair.h"


// Other includes
#include <vector>

#define PI  3.142

// struct for the robot config  
struct RobotConfig {

    float max_linear_vel;
    float max_angular_vel;

    float max_linear_acc;
    float max_angular_acc;

    float robot_radius;          // m
    float prediction_horizon;   // secs
    float dt;                    // secs 

};

// struct for robot state
struct RobotState {
    float x;        // m
    float y;        // m
    float z;        // m
    float theta;    // rad
};


// struct for control
struct Control {
    float v;  // m/s
    float w;  // rad/s
};


// define trajectory
// using Trajectory = std::vector<RobotState>;
// using RobotState = visual_interface::robotState;
// using RobotState = tf::Vector3;
using Trajectory = visual_interface::trajectory;
using TrajectoryPair = visual_interface::trajectoryPair;



// path prediction class
class PathPredictor {

    public:

        // constructor
        PathPredictor(ros::NodeHandle& nh, const float& prediction_horizon, const float& time_delta);

        // destructor
        ~PathPredictor() {};

        // publish trajectories
        void publishTrajectories(void);

    private:

        ros::NodeHandle nh_;
        // ROS services
        ros::Subscriber odom_subscriber_;
        ros::Subscriber user_cmd_subscriber_;
        ros::Subscriber optimal_cmd_subscriber_;
        ros::Publisher traj_publisher_;

        RobotConfig config_;
        RobotState current_state_;
        Control user_control_;
        Control optimal_control_;

        tf::TransformListener tf_listener_;
        tf::StampedTransform odom_to_camera_transform_;

    private:

        // Callbacks
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_data);

        void userCmdCallback(const geometry_msgs::Twist& cmd_vel);

        void optimalCmdCallback(const std_msgs::Float64MultiArray& velocities);

        // member functions
        TrajectoryPair computePredictedTraj(void);
          
        RobotState motionModel(const Control& vel_cmd, const RobotState& current_state);

        RobotState transformStates(const RobotState& state_in);

        void addStateToTrajectory(const RobotState& state_in, Trajectory& trajectory_out);

};