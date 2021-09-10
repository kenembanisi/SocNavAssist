#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros_falcon/falconForces.h"
#include "std_srvs/Empty.h"
#include <string>
#include "tf/tf.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"


#define PI 3.142

/**
 * falcon_haptic_control.h
 * 
 * This ROS node interfaces with the ros_falcon node to retrieve joystick data and send force data as well as
 * send velocity data to the robot
 * 
 */

/**
 * FalconNovintControl class
 */
class FalconNovintControl
{
    public: // public members

        // ros node handle
        ros::NodeHandle nh_;

        float base_orientation_ = 0.0f;
        float yaw_angle_ = 0.0f;
        float steering_angle_ = 0.0f;
        float raw_pedal_data_ = 0.0f;

        // control velocity variables
        float base_linear_vel_fwd_ = 0.0f;
        float base_linear_vel_bwd_ = 0.0f;
        float base_angular_vel_z_ = 0.0f;

        // falcon data
        float raw_x_pos_ = 0.0f;
        float raw_y_pos_ = 0.0f;
        float raw_z_pos_ = 0.0f;
        int button_pressed_ = 0;

        // falcon parameters
        float z_max_ = 0.10f; 
        float z_mid_ = 0.125f;
        float z_min_ = 0.172f;
        float z_buffer_ = 0.01f;
        float x_max_ = 0.05f; 
        float x_min_ = -0.05f;
        float x_buffer_ = 0.01f;
        float z_pos_range_ = z_min_ - z_max_;

        // car-like control parameters
        float max_wheel_angle_ = 70.0f;
        float veh_length_ = 2.5f;
        float steering_ratio_ = 1.2f;
        float max_yaw_angle_ = 120.0f * (180.0/PI);
        
        // vehicle motion limits
        float max_linear_vel_;
        float max_angular_vel_;

        // force variable
        float Ks_ = 12.0f; // 20
        // float Ks_ = 20.0f;
        // float Kf_ = 0.8f; // 0.75 or 1.20
        // float Kf_ = 1.2f;
        float Kf_x_ = 1.2f;
        float Kf_z_ = 0.5f;
        ros_falcon::falconForces force_fbk_;
        std::vector<float> centering_force_ = {0,0,0} ;
        std::vector<float> guidance_force_ = {0,0,0} ;
        float heading_delta_ = 0.0f;
        std::vector<float> control_delta_ = {0.0f,0.0f} ;

        // ros publishers and subscribers
        ros::Subscriber odom_subscriber_;
        ros::Publisher cmd_vel_publisher_;
        ros::Subscriber falcon_joy_subscriber_;
        ros::Publisher force_publisher_;
        ros::Subscriber heading_delta_subscriber_;
        ros::Subscriber control_delta_subscriber_;

        // ros services
        ros::ServiceClient unpause_sim_;
        ros::ServiceClient pause_sim_;

        // instantiate control mode
        std::string manual_mode_ = "pos-vel";
        std::string trial_condition_ = "MC";
        bool force_enabled_ = false;

        // custom objects
        geometry_msgs::Twist cmd_vel_;
        bool show_rearview = false;


    public: // public methods

        // Constructor
        FalconNovintControl(ros::NodeHandle &n);     

        // Callback functions
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_data);

        void falconCallback(const sensor_msgs::Joy::ConstPtr& falcon_data);

        void headingdeltaCallback(const std_msgs::Float32& heading_delta);

        void controldeltaCallback(const std_msgs::Float64MultiArray& control_delta);
        
        // Compute functions

        /**
        * Computes linear and angular velocities using the position-velocity mapping
        */
        void computePosVelControl();

        /**
        * Computes linear and angular velocities using the position-velocity mapping
        * with buffer
        */
        void computePosVelBufferControl();

        /**
        * Computes linear and angular velocities using the car-like kinematics mapping
        */
        void computeCarLikeControl();

        /**
        * Assigns the computed velocities and publishes to /cmd_vel
        */
        void commandVelocity();

        /**
        * Computes haptic forces and publishes to /falconForce
        */
        void commandForce();

};