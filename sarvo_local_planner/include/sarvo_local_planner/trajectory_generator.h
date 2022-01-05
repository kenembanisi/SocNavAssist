#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Twist2D.h"
#include "sarvo_msgs/Trajectory2D.h"
#include "sarvo_local_planner/utilities.h"
#include <ros/ros.h>

/*
Credits: https://github.com/ros-planning/navigation/blob/noetic-devel/base_local_planner/include/base_local_planner/simple_trajectory_generator.h

*/

namespace sarvo_local_planner {

using namespace sarvo_msgs;

class TrajectoryGenerator {


    public:

        TrajectoryGenerator();

        ~TrajectoryGenerator() {}

        TrajectoryGenerator(
            const double max_vel_x,
            const double max_vel_theta,
            const double max_acc_x,
            const double max_acc_theta,
            const double sim_time,
            const int vx_samples,
            const int vth_samples
        );
        
        
        void startNewIteration(const Pose2D pose,
            const Pose2D goal, 
            const Twist2D velocity, 
            const double dt);


        bool hasMoreSamples();


        Twist2D nextVelocity();


        Trajectory2D generateTrajectory(const Pose2D pose, 
            const Twist2D current_vel,
            const Twist2D target_vel);


        Twist2D computeNewVelocity(const Twist2D target_vel,
            const Twist2D current_vel,
            const double dt);


        double projectVelocity(const double target_vel,
            const double current_vel,
            const double max_accel,
            const double dt);


        std::vector<double> getTimeSteps(const Twist2D vel);


        Pose2D computeNextPose(const Pose2D current_pose,
            const Twist2D current_vel,
            const double dt);


    private:

        double max_vel_x_;
        double max_vel_theta_;
        double max_acc_x_;
        double max_acc_theta_;
        double sim_time_;
        int vx_samples_;
        int vth_samples_;
        bool discretize_by_time_;
        bool continued_accel_;
        double sim_granularity_ = 0.2;
        double angular_sim_granularity_;

        int next_sample_idx_;
        std::vector<Twist2D> velocity_samples_;

        Pose2D pose_;
        Twist2D vel_;

};








}