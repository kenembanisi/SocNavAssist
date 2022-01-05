#include <sarvo_local_planner/trajectory_generator.h>
#include <base_local_planner/velocity_iterator.h>


namespace sarvo_local_planner {


TrajectoryGenerator::TrajectoryGenerator(
    const double max_vel_x,
    const double max_vel_theta,
    const double max_acc_x,
    const double max_acc_theta,
    const double sim_time,
    const int vx_samples,
    const int vth_samples) :
    max_vel_x_(max_vel_x), max_vel_theta_(max_vel_theta),
    max_acc_x_(max_acc_x), max_acc_theta_(max_acc_theta),
    sim_time_(sim_time), vx_samples_(vx_samples), vth_samples_(vth_samples) {}


void TrajectoryGenerator::startNewIteration(const Pose2D pose, 
    const Pose2D goal, const Twist2D current_twist, const double dt)
{
    pose_ = pose;
    vel_ = current_twist;
    velocity_samples_.clear();
    next_sample_idx_ = 0;

    Twist2D max_vel;
    Twist2D min_vel;
    
    double max_vel_x = max_vel_x_;
    
    // limit search to the velocity to goal
    double distance = abs(pose, goal);

    max_vel_x = std::max( std::min( max_vel_x, distance / sim_time_ ), -max_vel_x_ );

    // using trajectory rollout approach, sampling the maximum velocity reachable in sim_time_
    max_vel.vx = projectVelocity(max_vel_x_, vel_.vx, max_acc_x_, sim_time_);
    max_vel.w = projectVelocity(max_vel_theta_, vel_.w, max_acc_theta_, sim_time_);
    min_vel.vx = projectVelocity(-max_vel_x_, vel_.vx, max_acc_x_, sim_time_);
    min_vel.w = projectVelocity(-max_vel_theta_, vel_.w, max_acc_theta_, sim_time_);

    // ROS_INFO("Curr_vel: [%f, %f] | Ori_Max_vel: [%f, %f] | Max_vel: [%f, %f] | Min_vel: [%f, %f]", 
    //     vel_.vx, vel_.w, max_vel_x_, max_vel_theta_, max_vel.vx, max_vel.w, min_vel.vx, min_vel.w);

    // // initialize velocity iterators
    base_local_planner::VelocityIterator x_it(min_vel.vx, max_vel.vx, vx_samples_);
    base_local_planner::VelocityIterator th_it(min_vel.w, max_vel.w, vth_samples_);
    Twist2D sample;
    for (; !x_it.isFinished(); x_it++){
        sample.vx = x_it.getVelocity();
        for (; !th_it.isFinished(); th_it++){
            sample.w = th_it.getVelocity();
            velocity_samples_.push_back(sample);
            // ROS_INFO("Sample: %f, %f", 
            //     sample.vx, sample.w);
        }
        th_it.reset();
    }

    // ROS_INFO(" ///////////////////////////////////// ");


}

double TrajectoryGenerator::projectVelocity(const double target_vel, const double current_vel,
    const double max_accel, const double dt)
{
    double new_vel;
    if (current_vel < target_vel){
        new_vel = current_vel + max_accel * dt;

        // ROS_INFO("[Curr, New, Acc, dt]: [%f, %f, %f, %f]", 
        //     current_vel, target_vel, max_accel, dt);

        return std::min(new_vel, target_vel);
    }
    else {
        new_vel = current_vel - max_accel * dt;
        return std::max(new_vel, target_vel);
    }
}



bool TrajectoryGenerator::hasMoreSamples()
{
    return next_sample_idx_ < velocity_samples_.size();
}


Twist2D TrajectoryGenerator::nextVelocity()
{
    return velocity_samples_[next_sample_idx_++];
}


Trajectory2D TrajectoryGenerator::generateTrajectory(const Pose2D start_pose, 
    const Twist2D current_vel, const Twist2D target_vel)
{
    Trajectory2D traj;

    Pose2D pose = start_pose;
    Twist2D vel = current_vel;
    double running_time = 0.0;
    std::vector<double> time_steps = getTimeSteps(target_vel);

    // for (auto dt : time_steps){
    //     ROS_INFO("%f", dt);
    // }
    // propagate trajectory
    for (auto dt : time_steps)
    {
        traj.poses.push_back(pose);
        // compute new trajectory based on acceleration
        vel = computeNewVelocity(target_vel, vel, dt);
        // advance the pose based on the latest velocity
        pose = computeNextPose(pose, vel, dt);
    }

    // ROS_INFO("////////////////////////");


    return traj;
}



std::vector<double> TrajectoryGenerator::getTimeSteps(const Twist2D vel)
{
    std::vector<double> steps;

    // if discretizing by time
    steps.resize(ceil(sim_time_ / sim_granularity_));

    std::fill(steps.begin(), steps.end(), sim_time_ / steps.size());
    // std::fill(steps.begin(), steps.end(), sim_time_ / 10);

    return steps;
}


Twist2D TrajectoryGenerator::computeNewVelocity(const Twist2D target_vel,
    const Twist2D current_vel, const double dt)
{
    Twist2D new_vel;
    new_vel.vx = projectVelocity(target_vel.vx, current_vel.vx, max_acc_x_, dt);
    new_vel.w = projectVelocity(target_vel.w, current_vel.w, max_acc_theta_, dt);
    return new_vel;
}


Pose2D TrajectoryGenerator::computeNextPose(const Pose2D current_pose,
    const Twist2D current_vel, const double dt)
{
    Pose2D next_pose;
    next_pose.x = current_pose.x + current_vel.vx * std::cos(current_pose.theta) * dt;
    next_pose.y = current_pose.y + current_vel.vx * std::sin(current_pose.theta) * dt;
    next_pose.z = current_pose.z;
    next_pose.theta = current_pose.theta + current_vel.w * dt;
    return next_pose;
}




}