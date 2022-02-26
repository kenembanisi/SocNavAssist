
#include "haptic_interface/haptic_interface.h"


/**
 * falcon_haptic_control.cpp
 * 
 * This ROS node interfaces with the ros_falcon node to retrieve joystick data and send force data as well as
 * send velocity data to the robot
 * 
 */

#define PI  3.142

// Constructor
FalconNovintControl::FalconNovintControl(ros::NodeHandle &nh) : nh_(nh){
    
    // initialize odometry subscriber
    odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/base_controller/odom", 5, &FalconNovintControl::odometryCallback, this);
    
    // initialize mobile_base velocity publisher
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/cmd_vel", 5);

    // initialize joystick subscriber
    falcon_joy_subscriber_ = nh_.subscribe("/falcon/joystick", 5, &FalconNovintControl::falconCallback, this);

    // initialize heading_delta subscriber
    heading_delta_subscriber_ = nh_.subscribe("/heading_delta", 5, &FalconNovintControl::headingdeltaCallback, this);

    // initialize control_delta subscriber
    control_delta_subscriber_ = nh_.subscribe("/control_delta", 5, &FalconNovintControl::controldeltaCallback, this);

    // initialize risk_level subscriber
    risk_level_subscriber_ = nh_.subscribe("/collision_risk_level", 5, &FalconNovintControl::riskLevelCallback, this);

    // initialize force publisher
    force_publisher_ = nh_.advertise<ros_falcon::falconForces>("/falconForce", 5);

    // initialize call to unpause simulation
    unpause_sim_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // initialize call to pause simulation
    pause_sim_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // get parameters from ROS param
    nh_.getParam("manual_mode", manual_mode_);
    nh_.getParam("trial_condition", trial_condition_);
    nh_.getParam("base_controller/linear/x/max_velocity", max_linear_vel_);
    nh_.getParam("base_controller/angular/z/max_velocity", max_angular_vel_);

    // set force_enabled
    // force_enabled_ = (trial_condition_ == "MC") ? false : true;
    force_enabled_ = false;
    if (trial_condition_ == "H" || trial_condition_ == "HV-T" || trial_condition_ == "HV-B" )
    {
        force_enabled_ = true;
    }

    // log info
    ROS_INFO("[HAPTIC_INTERFACE]: Novint Falcon Controller Initialized");

}

// Callback functions
void FalconNovintControl::odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    double roll, pitch, yaw;
    tf::Quaternion quat(odom_data->pose.pose.orientation.x,
                        odom_data->pose.pose.orientation.y,
                        odom_data->pose.pose.orientation.z,
                        odom_data->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    yaw_angle_ = yaw * (180/PI);  // yaw is in radians
}

void FalconNovintControl::falconCallback(const sensor_msgs::Joy::ConstPtr& falcon_data)
{
    raw_x_pos_ = falcon_data->axes[0];
    raw_y_pos_ = falcon_data->axes[1];
    raw_z_pos_ = falcon_data->axes[2];
    button_pressed_ = falcon_data->buttons[0];

    commandVelocity();
    commandForce();
}

void FalconNovintControl::headingdeltaCallback(const std_msgs::Float32& heading_delta)
{   
    heading_delta_ = heading_delta.data;

    // clip the value of heading delta
    heading_delta_ = (abs(heading_delta_) > PI) ? 0.0f : heading_delta_;
}

void FalconNovintControl::controldeltaCallback(const std_msgs::Float64MultiArray& control_delta)
{   
    control_delta_[0] = control_delta.data[0];
    control_delta_[1] = control_delta.data[1];

    // // clip the value of heading delta
    // heading_delta_ = (abs(heading_delta_) > PI) ? 0.0f : heading_delta_;

}

void FalconNovintControl::riskLevelCallback(const std_msgs::Float32& risk_level)
{   
    risk_level_ = risk_level.data;
}

// Compute functions

/**
* Computes linear and angular velocities using the position-velocity mapping
*/
void FalconNovintControl::computePosVelControl()
{
    // linear velocity
    if (raw_z_pos_ >= z_mid_) 
        { base_linear_vel_fwd_ = - (raw_z_pos_ - z_mid_) * 2 * max_linear_vel_/z_pos_range_; }
            // negative sign is there because its backward motion
    else 
        { base_linear_vel_fwd_ = (z_mid_ - raw_z_pos_) * 2 * max_linear_vel_/z_pos_range_; }

    // angular velocity
    base_angular_vel_z_ = -raw_x_pos_ * max_angular_vel_/x_max_; 
        // added -ve sign to address rotation direction
}

/**
* Computes linear and angular velocities using the position-velocity mapping
* with buffer
*/
void FalconNovintControl::computePosVelBufferControl()
{
    // linear velocity
    if ( abs(raw_z_pos_ - z_mid_) < 0.01) { base_linear_vel_fwd_ = 0.0; }  
    else {
        if (raw_z_pos_ >= z_mid_) 
            { base_linear_vel_fwd_ = - (raw_z_pos_ - z_mid_) * 2 * max_linear_vel_/(z_pos_range_ - z_buffer_); }
                // negative sign is there because its backward motion
        else 
            { base_linear_vel_fwd_ = (z_mid_ - raw_z_pos_) * 2 * max_linear_vel_/(z_pos_range_ - z_buffer_); }
    }

    // angular velocity
    if (raw_x_pos_ < 0.01){ base_angular_vel_z_ = 0.0; }
    else { 
        base_angular_vel_z_ = -raw_x_pos_ * max_angular_vel_/(x_max_ - x_buffer_);
            // added -ve sign to address rotation direction
    }                           
}

/**
* Computes linear and angular velocities using the car-like kinematics mapping
*/
void FalconNovintControl::computeCarLikeControl()
{
    // linear velocity
    float pos_range = z_min_ - z_max_;
    base_linear_vel_fwd_ = (z_min_ - raw_z_pos_) * max_linear_vel_/pos_range;

    // angular velocity
        // compute steering angle
    float steer_angle = raw_x_pos_ * max_yaw_angle_ / x_max_;
        // transform sttering angle to front wheel angle
    float fw_angle = steering_ratio_ * steer_angle;
        // bound the fw_angle
    if (fw_angle > max_wheel_angle_) { fw_angle = max_wheel_angle_; }
    if (fw_angle < max_wheel_angle_) { fw_angle = -max_wheel_angle_; }
        // calculate angular velocity
    base_angular_vel_z_ = -1 * (base_linear_vel_fwd_/veh_length_) * tan(fw_angle);
                    
}

/**
* Assigns the computed velocities and publishes to cmd_vel
*/
void FalconNovintControl::commandVelocity()
    {
        // select appropriate control mode
        if (manual_mode_ == "car-like") { this->computeCarLikeControl(); }
        if (manual_mode_ == "pos-vel") { this->computePosVelControl(); }
        if (manual_mode_ == "pos-vel-buffer") { this->computePosVelBufferControl(); }

        // set to cmd_vel
        cmd_vel_.linear.x = base_linear_vel_fwd_;
        cmd_vel_.angular.z = base_angular_vel_z_;

        // manage control engagement
        if (button_pressed_ != 4){      // button '4' is the middle button
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
        }

        // initialize the simulation by unpausing and then pausing
            // init empty request
        std_srvs::Empty srv;
        if (button_pressed_ == 8) {      // button '8' is the button to the left
            this->unpause_sim_.call(srv);
            ROS_INFO_ONCE("Simulation Initialized");
            this->pause_sim_.call(srv);
        }

        // toggle rearview display
        if (button_pressed_ == 1) {     // button '1' is the button to the right
            this->show_rearview = !this->show_rearview;
            nh_.setParam("toggle_rear_camera", this->show_rearview); }  


        // start up session
        if (button_pressed_ == 2) {     // button '2' is the button to the top
            this->unpause_sim_.call(srv);
            nh_.setParam("start_timer", true); }  

        // publish to cmd_vel
        // ROS_INFO("Velocity Published: [ %f, %f ]", cmd_vel_.linear.x, cmd_vel_.angular.z);

        this->cmd_vel_publisher_.publish(cmd_vel_);

        
    }

/**
* Computes haptic forces and publishes to /falconForce
*/
void FalconNovintControl::commandForce()
    {
        
        if (force_enabled_){
            if (cmd_vel_.linear.x > 0.0) { // apply force only when moving forward
                // compute the guidance forces
                // this->guidance_force_[0] = this->Kf_ * -this->heading_delta_;
                // ROS_INFO("Heading delta: [ %f ]", this->heading_delta_);

                this->guidance_force_[0] = -this->Kf_x_ * this->control_delta_[0];
                this->guidance_force_[2] = -this->Kf_z_ * this->control_delta_[1];
                // ROS_INFO("Control delta: [ %0.3f, %0.3f ]", this->control_delta_[0], this->control_delta_[1]);
            }
        }
        else {
            // compute centering force using f = K*(distance to center)
            this->centering_force_[0] = this->Ks_ * -(this->raw_x_pos_);
            this->centering_force_[2] = this->Ks_ * -(this->z_mid_ - this->raw_z_pos_);
        }
        
        // find resultant force
        this->force_fbk_.X = this->centering_force_[0] + this->guidance_force_[0];
        this->force_fbk_.Y = this->centering_force_[1] + this->guidance_force_[1];
        this->force_fbk_.Z = this->centering_force_[2] + this->guidance_force_[2];

        //publish the force
        force_publisher_.publish(this->force_fbk_);


        // ROS_INFO("Force applied: [ %f ]", this->force_fbk_.X);
        
    }


/**
 * main function
 */
int main(int argc, char **argv){

    // instantiate the node
    ros::init(argc, argv, "falcon_teleop_control");

    // ros node initialization
    ros::NodeHandle n;

    // instantiate haptic control object
    // FalconNovintControl base_control = FalconNovintControl(n);
    FalconNovintControl base_control(n);

    //  while (ros::ok())
    // {
    //     // run control
    //     base_control.commandVelocity();
    //     base_control.commandForce();
    //     ros::spinOnce();
    // }

    ros::spin();
    
    return 0;

    // ros::spin(); // keep loop alive until its shutdown

}


