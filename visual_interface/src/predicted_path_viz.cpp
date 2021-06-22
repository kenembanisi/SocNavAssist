
#include "visual_interface/predicted_path_viz.h"

// prediction class


// PathPredictor Constructor

PathPredictor::PathPredictor(ros::NodeHandle& nh, const float& prediction_horizon, const float& dt) : nh_(nh) {

    // initialize subscribers
    odom_subscriber_ = nh_.subscribe("/base_controller/odom", 10, 
                                &PathPredictor::odomCallback, this);
    
    user_cmd_subscriber_ = nh_.subscribe("/base_controller/cmd_vel", 10, 
                                &PathPredictor::userCmdCallback, this);

    optimal_cmd_subscriber_ = nh_.subscribe("/velocity_data", 10, 
                                &PathPredictor::optimalCmdCallback, this);
    
    // initialize publishers
    traj_publisher_ = nh_.advertise<TrajectoryPair>("/pred_trajectories", 1);

    // initialize the robot configuration
    nh_.getParam("base_controller/linear/x/max_velocity", config_.max_linear_vel);
    nh_.getParam("base_controller/angular/z/max_velocity", config_.max_angular_vel);
    nh_.getParam("base_controller/linear/x/max_acceleration", config_.max_linear_vel);
    nh_.getParam("base_controller/angular/z/max_acceleration", config_.max_angular_vel);
    config_.prediction_horizon = prediction_horizon;
    config_.dt = dt;
    
}


void PathPredictor::publishTrajectories(void) {

    // compute predicted trajectories
    TrajectoryPair out_trajectory = computePredictedTraj();

    // publish trajectories to topic "pred_trajectories"
    traj_publisher_.publish(out_trajectory);

}


void PathPredictor::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_data) {
    
    // retrieves the position states of the robot
    current_state_.x = odom_data->pose.pose.position.x;
    current_state_.y = odom_data->pose.pose.position.y;
    current_state_.z = 0.9f;

    double roll, pitch, yaw;
    tf::Quaternion quat(odom_data->pose.pose.orientation.x,
                        odom_data->pose.pose.orientation.y,
                        odom_data->pose.pose.orientation.z,
                        odom_data->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    // ROS_INFO("Yaw and Theta: [%lf, %lf]", yaw, yaw * (180/PI));

    current_state_.theta = yaw;  // theta is in radians

    // retrieve odom_to_camera transform
    try {
        tf_listener_.lookupTransform("/main_cam_color_optical_frame", "/odom",
                                    ros::Time(0), odom_to_camera_transform_);
    }
    catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
}


void PathPredictor::userCmdCallback(const geometry_msgs::Twist& cmd_vel) {
    // update user_control values
    user_control_.v = cmd_vel.linear.x;
    user_control_.w = cmd_vel.angular.z;
}


void PathPredictor::optimalCmdCallback(const std_msgs::Float64MultiArray& velocities) {
    // update optimal_control values
    optimal_control_.v = velocities.data[0];
    optimal_control_.w = velocities.data[1];
}


TrajectoryPair PathPredictor::computePredictedTraj(void) {

    TrajectoryPair out_trajectory;

    // initialize trajectories
    Trajectory user_traj, optimal_traj;
    // addStateToTrajectory(current_state_, user_traj);
    // addStateToTrajectory(current_state_, optimal_traj);

    // initialize next states
    RobotState next_state_user, next_state_user_trans;
    RobotState next_state_optimal, next_state_optimal_trans;

    float time = 0.0f;
    int counter = 0;
    RobotState current_state_user = current_state_;
    RobotState current_state_optimal = current_state_;
    
    while (time <= config_.prediction_horizon) {

        /// JUST FOR TEST
        // user_control_.v = 1.5f;
        // user_control_.w = 0.2f;
        // optimal_control_.v = 2.0f;
        // optimal_control_.w = -0.8f;
        ///
        // ROS_INFO("User velocity command, (v, w) : [%f, %f]", user_control_.v, user_control_.w);
        // ROS_INFO("Current state in /odom is: [%f, %f, %f]", current_state_.x, current_state_.y, current_state_.z);

        // compute the next state
        next_state_user = motionModel(user_control_, current_state_user);
        next_state_optimal = motionModel(optimal_control_, current_state_optimal);

        // transform states to robot frame
        next_state_user_trans = transformStates(next_state_user);
        next_state_optimal_trans = transformStates(next_state_optimal);

        // add new state to trajectory
        addStateToTrajectory(next_state_user_trans, user_traj);
        addStateToTrajectory(next_state_optimal_trans, optimal_traj);

        // increment time
        time += config_.dt;
        ++counter;      // number of states predicted

        // update current state
        current_state_user = next_state_user;
        current_state_optimal = next_state_optimal;
    }

    // add to the out_trajectory
    out_trajectory.user = user_traj;
    out_trajectory.optimal = optimal_traj;
    out_trajectory.count = counter;

    return out_trajectory;

}


RobotState PathPredictor::motionModel(const Control& vel_cmd, const RobotState& current_state) {

    RobotState new_state;

    // update theta first then use it to update x & y
    new_state.theta = current_state.theta + vel_cmd.w * config_.dt;
    
    // update x, y
    new_state.x = current_state.x + vel_cmd.v * std::cos(new_state.theta) * config_.dt;
    new_state.y = current_state.y + vel_cmd.v * std::sin(new_state.theta) * config_.dt;
    new_state.z = current_state.z;

    return new_state;
}


RobotState PathPredictor::transformStates(const RobotState& state_in) {
    // transforms from odom frame to camera frame
    RobotState state_out;
    tf::Vector3 state_in_vec3(state_in.x, state_in.y, state_in.z);
    tf::Vector3 state_out_vec3 = odom_to_camera_transform_ * state_in_vec3;

    state_out.x = state_out_vec3.getX();
    state_out.y = state_out_vec3.getY();
    state_out.z = state_out_vec3.getZ();

    // ROS_INFO("From: [%f, %f, %f] to [%f, %f, %f]", state_in.x, state_in.y, state_in.z, state_out.x, state_out.y, state_out.z);
    return state_out;
}

void PathPredictor::addStateToTrajectory(const RobotState& state_in, Trajectory& trajectory_out){
    // pushes the (x,y,z) state values to the equivalent arrays in the Trajectory type
    trajectory_out.x.push_back(state_in.x);
    trajectory_out.y.push_back(state_in.y);
    trajectory_out.z.push_back(state_in.z);    
}



int main(int argc, char** argv)
{
    // calling ROS:init
    ros::init(argc, argv, "trajectory_prediction");

    // instantiating ROS node handle
    ros::NodeHandle nh;

    float prediction_time = 2.0f;
    float dt = 0.1f;

    // instantiate the PathPredictor object
    PathPredictor path_predictor(nh, prediction_time, dt);

    while (ros::ok())
    {
        path_predictor.publishTrajectories();
        ros::spinOnce();
    }
    
    return 0;
}