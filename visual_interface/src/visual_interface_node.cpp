// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8MultiArray.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float32.h>
#include <gazebo_msgs/ModelStates.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

// Other includes
#include <time.h>
#include <string>
#include <vector>
#include <cmath> 
#include <random>

// custom defined msgs
#include <visual_interface/trajectoryPair.h>

// static const std::string OPENCV_WINDOW = "Image window";

// right bar
const int RIGHT_BASE_VERTEX_X1 = 1460; const int RIGHT_BASE_VERTEX_Y1 = 617;  // 635
const int RIGHT_BASE_VERTEX_X2 = 1610; const int RIGHT_BASE_VERTEX_Y2 = 653;  // 670
// left bar
const int LEFT_BASE_VERTEX_X1 = 1230; const int LEFT_BASE_VERTEX_Y1 = 617;
const int LEFT_BASE_VERTEX_X2 = 1380; const int LEFT_BASE_VERTEX_Y2 = 653;
// linear forward bar
const int LINEAR_FWD_BASE_VERTEX_X1 = 1400; const int LINEAR_FWD_BASE_VERTEX_Y1 = 580;
const int LINEAR_FWD_BASE_VERTEX_X2 = 1440; const int LINEAR_FWD_BASE_VERTEX_Y2 = 635;
// linear backward bar
const int LINEAR_BWD_BASE_VERTEX_X1 = 1400; const int LINEAR_BWD_BASE_VERTEX_Y1 = 635;
const int LINEAR_BWD_BASE_VERTEX_X2 = 1440; const int LINEAR_BWD_BASE_VERTEX_Y2 = 690;

// optimal right bar
const int OPTIMAL_RIGHT_BASE_VERTEX_X1 = 950; const int OPTIMAL_RIGHT_BASE_VERTEX_Y1 = 575;  // 635
const int OPTIMAL_RIGHT_BASE_VERTEX_X2 = 1150; const int OPTIMAL_RIGHT_BASE_VERTEX_Y2 = 600;  // 670
// optimal left bar
const int OPTIMAL_LEFT_BASE_VERTEX_X1 = 550; const int OPTIMAL_LEFT_BASE_VERTEX_Y1 = 575;
const int OPTIMAL_LEFT_BASE_VERTEX_X2 = 750; const int OPTIMAL_LEFT_BASE_VERTEX_Y2 = 600;


// optimal right bar
const int FORCE_RIGHT_BASE_VERTEX_X1 = 950; const int FORCE_RIGHT_BASE_VERTEX_Y1 = 575;  // 635
const int FORCE_RIGHT_BASE_VERTEX_X2 = 1050; const int FORCEL_RIGHT_BASE_VERTEX_Y2 = 600;  // 670
// optimal left bar
const int FORCE_LEFT_BASE_VERTEX_X1 = 550; const int FORCE_LEFT_BASE_VERTEX_Y1 = 575;
const int FORCE_LEFT_BASE_VERTEX_X2 = 750; const int FORCE_LEFT_BASE_VERTEX_Y2 = 600;
// linear forward bar
const int FORCE_FWD_BASE_VERTEX_X1 = 825; const int FORCE_FWD_BASE_VERTEX_Y1 = 450;
const int FORCE_FWD_BASE_VERTEX_X2 = 900; const int FORCE_FWD_BASE_VERTEX_Y2 = 550;
// linear backward bar
const int FORCE_BWD_BASE_VERTEX_X1 = 825; const int FORCE_BWD_BASE_VERTEX_Y1 = 675;
const int FORCE_BWD_BASE_VERTEX_X2 = 900; const int FORCE_BWD_BASE_VERTEX_Y2 = 775;

class VisualInterface
{
  
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber fwd_image_sub_;
    image_transport::Subscriber bwd_image_sub_;
    //   image_transport::Publisher image_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber proxemics_sub_;
    ros::Subscriber proxemics_score_sub_;
    ros::Subscriber predicted_traj_sub_;
    ros::Subscriber heading_delta_sub_;
    ros::Subscriber control_delta_sub_;
    ros::Subscriber v_optimal_sub_;
    cv_bridge::CvImagePtr fwd_img_ptr;
    cv_bridge::CvImagePtr bwd_img_ptr;
    int angular_vel_left, angular_vel_right, linear_vel;
    int optimal_angular_vel_left, optimal_angular_vel_right;
    int force_left, force_right, force_linear;
    float max_linear_vel, max_angular_vel, min_linear_vel, min_angular_vel;
    time_t start_time;
    bool show_rearview = false;
    bool start_timer = false;
    bool show_debug_bars = false;
    bool distracted_mode = false;
    int proxemics_state_[2];
    std::string trial_category;
    std::string trial_condition;
    std::string trial_condition_name;
    std::string trial_number;
    std::string trial_mode;
    std::string behavior_title;
    std::string task_title;
    std::string window_name;
    visual_interface::trajectory user_pred_traj_;
    visual_interface::trajectory optimal_pred_traj_;
    int pred_trajectory_size_;
    float heading_delta_ = 0.0f;
    std_msgs::Float64MultiArray control_delta_;;
    float proxemics_score_ = 0.0f;
    float operator_vel_[2] = { 0.0f, 0.0f };
    float optimal_velocity_[2] = { 0.0f, 0.0f };
    std::vector<int> first_number_ = { 23, 19, 26, 46, 22, 47, 6, 63, 35, 28,
                                       35, 20, 45, 41, 15};
    std::vector<int> second_number_ = { 5, 13, 17, 38, 76, 12, 61, 57, 36, 17,
                                        28, 75, 29, 32, 57};
    int idx1 = 0, idx2 = 0;

  public:
    // CONSTRUCTOR
    VisualInterface() : it_(nh_)
    {   
        // Subscribe to forward image stream
        this->fwd_image_sub_ = it_.subscribe("/main_cam_wide/color/image_raw", 1, &VisualInterface::fwdImageCb, this); // forward view
        
        // Subscribe to rear image stream
        this->bwd_image_sub_ = it_.subscribe("/rear_cam/color/image_raw", 1, &VisualInterface::bwdImageCb, this); // rear view
 
        // Subscribe to the base_controller velocity
        this->cmd_vel_sub_ = nh_.subscribe("/base_controller/cmd_vel", 10, &VisualInterface::velocityCb, this);

        // Subscribe to the proxemics topic
        this->proxemics_sub_ = nh_.subscribe("/proxemics_states", 10, &VisualInterface::proxemicsCb, this);

        // Subscribe to the proxemics score topic
        this->proxemics_score_sub_ = nh_.subscribe("/proxemics_score", 10, &VisualInterface::proxemicsScoreCb, this);

        // Subscribe to the predicted trajectories topic
        this->predicted_traj_sub_ = nh_.subscribe("/pred_trajectories", 10, &VisualInterface::predictedTrajCb, this);

        // Subscribe to heading delta topic
        this->heading_delta_sub_ = nh_.subscribe("/heading_delta", 10, &VisualInterface::headingDeltaCb, this);

        // Subscribe to heading delta topic
        this->control_delta_sub_ = nh_.subscribe("/control_delta", 10, &VisualInterface::controlDeltaCb, this);

        // Subscribe to v_opt (optimal velocity command) topic
        this->v_optimal_sub_ = nh_.subscribe("/velocity_data", 10, &VisualInterface::optimalVelCb, this);

        // Get parameters
        nh_.getParam("base_controller/linear/x/max_velocity", this->max_linear_vel);
        nh_.getParam("base_controller/linear/x/min_velocity", this->min_linear_vel);
        nh_.getParam("base_controller/angular/z/max_velocity", this->max_angular_vel);
        nh_.getParam("base_controller/angular/z/min_velocity", this->min_angular_vel);
        nh_.getParam("toggle_rear_camera", this->show_rearview);
        nh_.getParam("trial_number", this->trial_number);
        nh_.getParam("trial_condition", this->trial_condition);
        nh_.getParam("trial_category", this->trial_category);
        nh_.getParam("trial_mode", this->trial_mode);
        nh_.getParam("behavior", this->behavior_title);
        nh_.getParam("task_objective", this->task_title);
        nh_.getParam("distracted_mode", this->distracted_mode);


        // Set initial values for secondary task idx
        idx1 = (std::rand() % 10);
        idx2 = (std::rand() % 10);



        // cv::namedWindow(OPENCV_WINDOW);
        window_name = this->trial_mode + "   |   Category " + this->trial_category + "   |   " + this->trial_condition + "   |   " + this->trial_number;
        // const cv::String window_name = "Category#1-1/2";
        cv::namedWindow(window_name);

        ROS_INFO("Initialized Visual Interface Node");
    }

    // DESTRUCTOR
    ~VisualInterface()
    {
        cv::destroyWindow(window_name);
    }

    // Callback method
    void fwdImageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            this->fwd_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }

    void bwdImageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            this->bwd_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }

    void velocityCb(const geometry_msgs::Twist& msg)
    {
        // linear velocity
        if (msg.linear.x >= 0.0) {
            this->linear_vel = LINEAR_FWD_BASE_VERTEX_Y2 - (LINEAR_FWD_BASE_VERTEX_Y2 - LINEAR_FWD_BASE_VERTEX_Y1) * (msg.linear.x/this->max_linear_vel);
        }
        else {
            this->linear_vel = LINEAR_BWD_BASE_VERTEX_Y1 - (LINEAR_BWD_BASE_VERTEX_Y2 - LINEAR_BWD_BASE_VERTEX_Y1) * (msg.linear.x/this->max_linear_vel);
            // ROS_INFO("Linear pixel value is %d ", this->linear_vel);
        }
        

        if (msg.angular.z <= 0.0)  {
            // right angular velocity
            this->angular_vel_right = RIGHT_BASE_VERTEX_X1 - (RIGHT_BASE_VERTEX_X2 - RIGHT_BASE_VERTEX_X1) * (msg.angular.z/this->max_angular_vel); 
            this->angular_vel_left = LEFT_BASE_VERTEX_X2; 
            }

        else { 
            // left angular velocity
            this->angular_vel_left = LEFT_BASE_VERTEX_X2 - (LEFT_BASE_VERTEX_X2 - LEFT_BASE_VERTEX_X1) * (msg.angular.z/this->max_angular_vel); 
            this->angular_vel_right = RIGHT_BASE_VERTEX_X1; 
            };
        
        // store operator linear and angular velocity
        operator_vel_[0] = msg.linear.x;
        operator_vel_[1] = msg.angular.z;

        // std::cout << "Max and current angular velocity are: [" << msg.angular.z << " ," << this->max_angular_vel << "] \n" ;
        
    }

    void proxemicsCb(const std_msgs::Int8MultiArray& msg){
        proxemics_state_[0] = msg.data[0];
        proxemics_state_[1] = msg.data[1];
    }

    void proxemicsScoreCb(const std_msgs::Float32& msg){
        proxemics_score_ += msg.data;
        // ROS_INFO("Proxemics score is [%0.3f]", proxemics_score_);
    }

    void optimalVelCb(const std_msgs::Float64MultiArray& velocities) {
        
        // linear velocity
        // if (msg.linear.x >= 0.0) {
        //     this->linear_vel = LINEAR_FWD_BASE_VERTEX_Y2 - (LINEAR_FWD_BASE_VERTEX_Y2 - LINEAR_FWD_BASE_VERTEX_Y1) * (msg.linear.x/this->max_linear_vel);
        // }
        // else {
        //     this->linear_vel = LINEAR_BWD_BASE_VERTEX_Y1 - (LINEAR_BWD_BASE_VERTEX_Y2 - LINEAR_BWD_BASE_VERTEX_Y1) * (msg.linear.x/this->max_linear_vel);
        //     // ROS_INFO("Linear pixel value is %d ", this->linear_vel);
        // }

        // if (velocities.data[1] <= 0.0)  {
        //     // right angular velocity
        //     this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1 - (OPTIMAL_RIGHT_BASE_VERTEX_X2 - OPTIMAL_RIGHT_BASE_VERTEX_X1) * (velocities.data[1]/this->max_angular_vel); 
        //     this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2; 
        //     // ROS_INFO("Angular pixel value to the right is %d ", this->optimal_angular_vel_right);
        //     }

        // else { 
        //     // left angular velocity
        //     this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2 - (OPTIMAL_LEFT_BASE_VERTEX_X2 - OPTIMAL_LEFT_BASE_VERTEX_X1) * (velocities.data[1]/this->max_angular_vel); 
        //     this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1; 
        //     // ROS_INFO("Angular pixel value to the left is %d ", this->optimal_angular_vel_left);
        //     };          
        
        // update optimal_velocity values
        this->optimal_velocity_[0] = velocities.data[0];
        this->optimal_velocity_[1] = velocities.data[1];
    }

    void predictedTrajCb(const visual_interface::trajectoryPair& msg){
        user_pred_traj_ = msg.user;
        optimal_pred_traj_ = msg.optimal;
        pred_trajectory_size_ = msg.count;

        // ROS_INFO("Size of traj: [%d]", pred_trajectory_size_);

        // displayVisual();
    }

    void headingDeltaCb(const std_msgs::Float32& msg){
        heading_delta_ = msg.data;

        if (operator_vel_[0] > 0.0) 
        {
            if (std::abs(heading_delta_) > 0.1) 
            {
                if (heading_delta_ <= 0.0)  
                {
                    // heading_delta_ = std::max(double(heading_delta_), -1.732);
                    if (heading_delta_ <= -1.732) heading_delta_ = 0.0;
                    // right angular velocity
                    this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1 - (OPTIMAL_RIGHT_BASE_VERTEX_X2 - OPTIMAL_RIGHT_BASE_VERTEX_X1) * (heading_delta_/1.732); 
                    this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2; 
                    // ROS_INFO("(RIGHT) Heading delta is %0.2f ", heading_delta_);
                    }

                else { 
                    // heading_delta_ = std::min(double(heading_delta_), 1.732);
                    if (heading_delta_ >= 1.732) heading_delta_ = 0.0;
                    // left angular velocity
                    this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2 - (OPTIMAL_LEFT_BASE_VERTEX_X2 - OPTIMAL_LEFT_BASE_VERTEX_X1) * (heading_delta_/1.732); 
                    this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1; 
                    // ROS_INFO("(LEFT) Heading delta is %0.2f ", heading_delta_);
                    // ROS_INFO("Angular pixel value to the left is %d ", this->optimal_angular_vel_left);
                    };    
                }
            else {
                this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1; 
                this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2; 
            }
        }
        else {
            this->optimal_angular_vel_right = OPTIMAL_RIGHT_BASE_VERTEX_X1; 
            this->optimal_angular_vel_left = OPTIMAL_LEFT_BASE_VERTEX_X2; 
        }
    }

    void controlDeltaCb( const std_msgs::Float64MultiArray& msg){
        // control_delta_ = msg.data;

        // force in x direction
        if (msg.data[1] >= 0.0) {
            this->force_linear = FORCE_FWD_BASE_VERTEX_Y2 - (FORCE_FWD_BASE_VERTEX_Y2 - FORCE_FWD_BASE_VERTEX_Y1) * (msg.data[1] / 2.0);
        }
        else {
            this->force_linear = FORCE_BWD_BASE_VERTEX_Y1 - (FORCE_BWD_BASE_VERTEX_Y2 - FORCE_BWD_BASE_VERTEX_Y1) * (msg.data[1] / 2.0);
            // ROS_INFO("Linear pixel value is %d ", this->linear_vel);
        }
        

        if (msg.data[0] <= 0.0)  {
            // right angular velocity
            this->force_right = FORCE_RIGHT_BASE_VERTEX_X1 - (FORCE_RIGHT_BASE_VERTEX_X2 - FORCE_RIGHT_BASE_VERTEX_X1) * (msg.data[0] / 2.0); 
            this->force_left = FORCE_LEFT_BASE_VERTEX_X2; 
            }

        else { 
            // left angular velocity
            this->force_left = FORCE_LEFT_BASE_VERTEX_X2 - (FORCE_LEFT_BASE_VERTEX_X2 - FORCE_LEFT_BASE_VERTEX_X1) * (msg.data[0] / 2.0); 
            this->force_right = FORCE_RIGHT_BASE_VERTEX_X1; 
            };

        // ROS_INFO("Control delta is [%0.3f, %0.3f] | Force display [%d, %d, %d] ", msg.data[0], msg.data[1],
            // this->force_linear, this->force_right, this->force_left);
        
    }

    void drawOperatorSpeedBars() {

        // Draw the right angular speed bars
        int RIGHT_LEVEL_VERTEX_X1 = 1460; int RIGHT_LEVEL_VERTEX_Y1 = 617;
        int RIGHT_LEVEL_VERTEX_X2 = this->angular_vel_right; int RIGHT_LEVEL_VERTEX_Y2 = 653;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_BASE_VERTEX_X1, RIGHT_BASE_VERTEX_Y1), 
                    cv::Point(RIGHT_BASE_VERTEX_X2, RIGHT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_LEVEL_VERTEX_X1, RIGHT_LEVEL_VERTEX_Y1), 
                    cv::Point(RIGHT_LEVEL_VERTEX_X2, RIGHT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 117, 163, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);
        cv::putText(this->fwd_img_ptr->image, "Right", cv::Point(1515, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

        // Draw the left angular speed bars
        int LEFT_LEVEL_VERTEX_X1 = this->angular_vel_left; int LEFT_LEVEL_VERTEX_Y1 = 617;
        int LEFT_LEVEL_VERTEX_X2 = 1380; int LEFT_LEVEL_VERTEX_Y2 = 653;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LEFT_BASE_VERTEX_X1, LEFT_BASE_VERTEX_Y1), 
                    cv::Point(LEFT_BASE_VERTEX_X2, LEFT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LEFT_LEVEL_VERTEX_X1, LEFT_LEVEL_VERTEX_Y1), 
                    cv::Point(LEFT_LEVEL_VERTEX_X2, LEFT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 117, 163, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);
        cv::putText(this->fwd_img_ptr->image, "Left", cv::Point(1290, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

        // Draw the linear velocity bars
        int LINEAR_LEVEL_VERTEX_X1 = 1400; int LINEAR_LEVEL_VERTEX_Y1 = this->linear_vel;
        int LINEAR_LEVEL_VERTEX_X2 = 1440; int LINEAR_LEVEL_VERTEX_Y2 = 635;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LINEAR_FWD_BASE_VERTEX_X1, LINEAR_FWD_BASE_VERTEX_Y1), 
                    cv::Point(LINEAR_BWD_BASE_VERTEX_X2, LINEAR_BWD_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LINEAR_LEVEL_VERTEX_X1, LINEAR_LEVEL_VERTEX_Y1), 
                    cv::Point(LINEAR_LEVEL_VERTEX_X2, LINEAR_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 117, 163, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);

    }

    void drawOptimalSpeedBars() {

        // Draw the right angular speed bars
        int RIGHT_LEVEL_VERTEX_X1 = 950; int RIGHT_LEVEL_VERTEX_Y1 = 575;
        int RIGHT_LEVEL_VERTEX_X2 = this->optimal_angular_vel_right; int RIGHT_LEVEL_VERTEX_Y2 = 600;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_BASE_VERTEX_X1, RIGHT_BASE_VERTEX_Y1), 
                    cv::Point(RIGHT_BASE_VERTEX_X2, RIGHT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_LEVEL_VERTEX_X1, RIGHT_LEVEL_VERTEX_Y1), 
                    cv::Point(RIGHT_LEVEL_VERTEX_X2, RIGHT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 40, 40, 200 ),
                    cv::FILLED, 
                    cv::LINE_8);
        // cv::putText(this->fwd_img_ptr->image, "Right", cv::Point(1515, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

        // Draw the left angular speed bars
        int LEFT_LEVEL_VERTEX_X1 = this->optimal_angular_vel_left; int LEFT_LEVEL_VERTEX_Y1 = 575;
        int LEFT_LEVEL_VERTEX_X2 = 750; int LEFT_LEVEL_VERTEX_Y2 = 600;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LEFT_BASE_VERTEX_X1, LEFT_BASE_VERTEX_Y1), 
                    cv::Point(LEFT_BASE_VERTEX_X2, LEFT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LEFT_LEVEL_VERTEX_X1, LEFT_LEVEL_VERTEX_Y1), 
                    cv::Point(LEFT_LEVEL_VERTEX_X2, LEFT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 40, 40, 200 ),
                    cv::FILLED, 
                    cv::LINE_8);
        // cv::putText(this->fwd_img_ptr->image, "Left", cv::Point(1290, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

    }

    void drawForceBars() {
        // Draw the right angular speed bars
        int FORCE_RIGHT_LEVEL_VERTEX_X1 = 950; int FORCE_RIGHT_LEVEL_VERTEX_Y1 = 575;
        int FORCE_RIGHT_LEVEL_VERTEX_X2 = this->force_right; int FORCE_RIGHT_LEVEL_VERTEX_Y2 = 600;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_BASE_VERTEX_X1, RIGHT_BASE_VERTEX_Y1), 
                    cv::Point(RIGHT_BASE_VERTEX_X2, RIGHT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(FORCE_RIGHT_LEVEL_VERTEX_X1, FORCE_RIGHT_LEVEL_VERTEX_Y1), 
                    cv::Point(FORCE_RIGHT_LEVEL_VERTEX_X2, FORCE_RIGHT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 117, 163, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);
        cv::putText(this->fwd_img_ptr->image, "Right", cv::Point(1515, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

        // Draw the left angular speed bars
        int FORCE_LEFT_LEVEL_VERTEX_X1 = this->force_left; int FORCE_LEFT_LEVEL_VERTEX_Y1 = 575;
        int FORCE_LEFT_LEVEL_VERTEX_X2 = 750; int FORCE_LEFT_LEVEL_VERTEX_Y2 = 600;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(FORCE_LEFT_BASE_VERTEX_X1, FORCE_LEFT_BASE_VERTEX_Y1), 
                    cv::Point(FORCE_LEFT_BASE_VERTEX_X2, FORCE_LEFT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(FORCE_LEFT_LEVEL_VERTEX_X1, FORCE_LEFT_LEVEL_VERTEX_Y1), 
                    cv::Point(FORCE_LEFT_LEVEL_VERTEX_X2, FORCE_LEFT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 117, 163, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);
        cv::putText(this->fwd_img_ptr->image, "Left", cv::Point(1290, 690), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar( 0, 0, 0 ), 2, false);

        // // Draw the linear velocity bars
        // int FORCE_LINEAR_LEVEL_VERTEX_X1 = 1400; int FORCE_LINEAR_LEVEL_VERTEX_Y1 = this->force_linear;
        // int FORCE_LINEAR_LEVEL_VERTEX_X2 = 1440; int FORCE_LINEAR_LEVEL_VERTEX_Y2 = 635;
        // cv::rectangle(this->fwd_img_ptr->image, 
        //             cv::Point(LINEAR_FWD_BASE_VERTEX_X1, LINEAR_FWD_BASE_VERTEX_Y1), 
        //             cv::Point(LINEAR_BWD_BASE_VERTEX_X2, LINEAR_BWD_BASE_VERTEX_Y2), 
        //             cv::Scalar( 0, 0, 0 ),
        //             cv::FILLED, 
        //             cv::LINE_8);

        // cv::rectangle(this->fwd_img_ptr->image, 
        //             cv::Point(FORCE_LINEAR_LEVEL_VERTEX_X1, FORCE_LINEAR_LEVEL_VERTEX_Y1), 
        //             cv::Point(FORCE_LINEAR_LEVEL_VERTEX_X2, FORCE_LINEAR_LEVEL_VERTEX_Y2), 
        //             cv::Scalar( 117, 163, 8 ),
        //             cv::FILLED, 
        //             cv::LINE_8);
    }

    void drawTimer() {

        // compute time
        int time_p = (int)difftime(time(0), this->start_time);
        std::string time_passed = std::to_string(time_p);

        // display timer
        // cv::putText(this->fwd_img_ptr->image, time_passed, cv::Point(1520, 65), 
        //                     cv::FONT_HERSHEY_DUPLEX, 2, 
        //                     cv::Scalar( 0, 0, 0 ), 2, false);

        cv::putText(this->fwd_img_ptr->image, time_passed, cv::Point(1520, 60), 
                            cv::FONT_HERSHEY_DUPLEX, 1.7, 
                            cv::Scalar( 0, 0, 0 ), 2, false);

        cv::String time = "Time:";
        cv::putText(this->fwd_img_ptr->image, time, cv::Point(1400, 55), 
                            cv::FONT_HERSHEY_DUPLEX, 1.3, 
                            cv::Scalar( 0, 0, 0 ), 2, false);
    }

    void drawScenarioTitle() {

        // display proximity score
        cv::String safety_score = "Proximity Score";
        cv::putText(this->fwd_img_ptr->image, safety_score, cv::Point(80, 30), 
                            cv::FONT_HERSHEY_DUPLEX, 0.75, 
                            cv::Scalar( 0, 0, 0 ), 2, false);
        
        int RIGHT_BASE_VERTEX_X1 = 60; int RIGHT_BASE_VERTEX_Y1 = 50;
        int RIGHT_BASE_VERTEX_X2 = 240; int RIGHT_BASE_VERTEX_Y2 = 100;
        int RIGHT_LEVEL_VERTEX_X1 = 60; int RIGHT_LEVEL_VERTEX_Y1 = 50;
        int RIGHT_LEVEL_VERTEX_X2; 
        int RIGHT_LEVEL_VERTEX_Y2 = 100;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_BASE_VERTEX_X1, RIGHT_BASE_VERTEX_Y1), 
                    cv::Point(RIGHT_BASE_VERTEX_X2, RIGHT_BASE_VERTEX_Y2), 
                    cv::Scalar( 0, 0, 0 ),
                    cv::FILLED, 
                    cv::LINE_8);
        
        float max_score = 100.0f;
        if (proxemics_score_ < max_score)
            RIGHT_LEVEL_VERTEX_X2 = RIGHT_LEVEL_VERTEX_X1 + (RIGHT_BASE_VERTEX_X2 - RIGHT_LEVEL_VERTEX_X1) * (proxemics_score_ / max_score);
        else RIGHT_LEVEL_VERTEX_X2 = RIGHT_BASE_VERTEX_X2;

        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_LEVEL_VERTEX_X1, RIGHT_LEVEL_VERTEX_Y1), 
                    cv::Point(RIGHT_LEVEL_VERTEX_X2, RIGHT_LEVEL_VERTEX_Y2), 
                    cv::Scalar( 90, 120, 8 ),
                    cv::FILLED, 
                    cv::LINE_8);
        cv::putText(this->fwd_img_ptr->image, std::to_string(int(proxemics_score_)), cv::Point(260, 80), 
                            cv::FONT_HERSHEY_DUPLEX, 1.1, 
                            cv::Scalar( 0, 0, 0 ), 2, false);
        // }

        // display behavior
        cv::String scenario_title = "A: ";
        if (this->behavior_title != "none") {
            cv::putText(this->fwd_img_ptr->image, scenario_title+this->behavior_title, cv::Point(540, 55), 
                                cv::FONT_HERSHEY_DUPLEX, 1.3, 
                                cv::Scalar( 100, 10, 10 ), 2, false);
        }

        // if (this->behavior_title != "none" && this->trial_mode == "testing") {
        //     if (this->behavior_title == "goal_aligned") {
        //         cv::String behavior = "option 1";
        //         cv::putText(this->fwd_img_ptr->image, scenario_title+behavior, cv::Point(540, 55), 
        //                             cv::FONT_HERSHEY_DUPLEX, 1.3, 
        //                             cv::Scalar( 100, 10, 10 ), 2, false);
        //     }
        //     else {
        //         cv::String behavior = "option 2";
        //         cv::putText(this->fwd_img_ptr->image, scenario_title+behavior, cv::Point(540, 55), 
        //                             cv::FONT_HERSHEY_DUPLEX, 1.3, 
        //                             cv::Scalar( 100, 10, 10 ), 2, false);
        //     }
        // }


        // display task objective
        if (this->task_title != "none") {
            cv::String task_title = "T: ";
            cv::putText(this->fwd_img_ptr->image, task_title+this->task_title, cv::Point(930, 55), 
                                cv::FONT_HERSHEY_DUPLEX, 1.3, 
                                cv::Scalar( 0, 100, 0 ), 2, false);
        }
    }

    void drawSideIndicators() {

        // get the proxemics state
        int fill_rect = (proxemics_state_[0] == 1) ? cv::FILLED : 0;

        int LEFT_INDICATOR_PANEL_X1 = 0; int LEFT_INDICATOR_PANEL_Y1 = 0;
        int LEFT_INDICATOR_PANEL_X2 = 50; int LEFT_INDICATOR_PANEL_Y2 = 800;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(LEFT_INDICATOR_PANEL_X1, LEFT_INDICATOR_PANEL_Y1), 
                    cv::Point(LEFT_INDICATOR_PANEL_X2, LEFT_INDICATOR_PANEL_Y2), 
                    cv::Scalar( 66, 66, 245, 0.6 ),
                    fill_rect, 
                    cv::LINE_8);

        int RIGHT_INDICATOR_PANEL_X1 = 1630; int RIGHT_INDICATOR_PANEL_Y1 = 0;
        int RIGHT_INDICATOR_PANEL_X2 = 1700; int RIGHT_INDICATOR_PANEL_Y2 = 800;
        cv::rectangle(this->fwd_img_ptr->image, 
                    cv::Point(RIGHT_INDICATOR_PANEL_X1, RIGHT_INDICATOR_PANEL_Y1), 
                    cv::Point(RIGHT_INDICATOR_PANEL_X2, RIGHT_INDICATOR_PANEL_Y2), 
                    cv::Scalar( 66, 66, 245, 1.0),
                    fill_rect, 
                    cv::LINE_8);
    }

    void drawRearView(){
        // resize the rear view image
            // create a new image
        // cv::Mat rear_image(500, 300, CV_8UC4, cv::Scalar(0, 0, 50));
        cv::Mat rear_image = this->bwd_img_ptr->image.clone();

        // resize
        cv::resize(rear_image, rear_image, cv::Size(300, 160));

        // place on main image
        rear_image.copyTo(this->fwd_img_ptr->image(cv::Rect(750, 10, rear_image.cols, rear_image.rows)));


    }
    
    void drawPredictedTrajectories(){        

        if (operator_vel_[0] > 0.0) {   // show trajectories only if linear velocity is +ve
        // float x = 5.0f, y = 0.f;
            for (int i = 0; i < pred_trajectory_size_; ++i) 
            {  
                float threshold = 0.035; // formerly 0.1 and 0.05

                if (this->trial_condition == "HV-T" || this->trial_condition == "V-T") 
                {
                    // v_opt predicted trajectories
                    if (abs(heading_delta_) > threshold)
                    {
                        cv::circle(this->fwd_img_ptr->image, 
                            tranformToPixel(optimal_pred_traj_.x[i], 
                                            optimal_pred_traj_.y[i], 
                                            optimal_pred_traj_.z[i]), 
                            computePointSize(optimal_pred_traj_.z[i]), 
                            cv::Scalar( 20, 15, 245, 0.6 ), cv::FILLED);
                        // ROS_INFO("Getting in here!");
                    }
                }    
            

                // operator-based predicted trajectories
                cv::circle(this->fwd_img_ptr->image, 
                            tranformToPixel(user_pred_traj_.x[i], 
                                            user_pred_traj_.y[i], 
                                            user_pred_traj_.z[i]), 
                            computePointSize(user_pred_traj_.z[i]), 
                            cv::Scalar( 20, 245, 15, 0.6 ), cv::FILLED);
            }
        }

        // ROS_INFO("Position values for last point: [%f, %f, %f]", user_pred_traj_.x[0], user_pred_traj_.y[0], user_pred_traj_.z[0]);
        // ROS_INFO("Pixel values for last point: [%d, %d]", tranformToPixel(user_pred_traj_.x[0], user_pred_traj_.y[0], user_pred_traj_.z[0]).x, 
        //                                                      tranformToPixel(user_pred_traj_.x[0], user_pred_traj_.y[0], user_pred_traj_.z[0]).y);
        
        // cv::circle(this->fwd_img_ptr->image, tranformToPixel(x, y, z), size, 
        //             cv::Scalar( 20, 15, 245, 0.6 ), cv::FILLED);

        // for (int i = 0; i < points.size(); ++i ){
        // cv::circle(this->fwd_img_ptr->image, tranformToPixel(points[i][0], points[i][1], points[i][2]), size, 
        //             cv::Scalar( 20, 15, 245, 0.6 ), cv::FILLED);
        // }
    }

    cv::Point tranformToPixel(double& x, double& y, double& z) {

        // cv::Point pixel_value( (926*(6.31 + x)/(9.55 + y) + 640), (926*(1.172 - z)/(9.55 + y) + 360) ); standard fov
        // cv::Point pixel_value( (546*(6.31 + x)/(9.55 + y) + 840), (546*(1.172 - z)/(9.55 + y) + 360) ); // wide fov using gazebo world frame
        cv::Point pixel_value( (546*x/z + 840), (546*y/z + 360) ); // wide fov

        return pixel_value;
    }
    
    int computePointSize(double& dist_in_camera_frame) {

        // scale the size of the point with max=10 and min=3
        int max_distance = 3;    // m

        int size = 10 - (dist_in_camera_frame/max_distance) * 10;    // this rounds to integer value
        if (size > 10) {size = 10; } 
        else if (size < 3) { size = 3;}

        return size;
    }

    void drawTopViewMap(){

        cv::circle(this->fwd_img_ptr->image, 
                    cv::Point(250, 635), 
                    90, 
                    cv::Scalar( 195, 195, 195, 0.4 ), cv::FILLED);
    }

    void drawSecondaryTask(){

        // temp:
        cv::Point p1(60, 600), p2(280, 600), p3(280, 700), p4(60, 700);
        int thickness = 3;
        cv::Scalar font_color = cv::Scalar(0, 160, 0);

        cv::line(this->fwd_img_ptr->image, 
                 p1, p2, font_color,
                 thickness, cv::LINE_8);

        cv::line(this->fwd_img_ptr->image, 
                 p2, p3, font_color,
                 thickness, cv::LINE_8);

        cv::line(this->fwd_img_ptr->image, 
                 p3, p4, font_color,
                 thickness, cv::LINE_8);

        cv::line(this->fwd_img_ptr->image, 
                 p1, p4, font_color,
                 thickness, cv::LINE_8);


        cv::putText(this->fwd_img_ptr->image, std::to_string(first_number_[idx1]), cv::Point(75, 670), 
                cv::FONT_HERSHEY_DUPLEX, 1.75, 
                font_color, thickness, false);

        cv::putText(this->fwd_img_ptr->image, "+", cv::Point(150, 670), 
                cv::FONT_HERSHEY_DUPLEX, 1.75, 
                font_color, thickness, false);
        
        cv::putText(this->fwd_img_ptr->image, std::to_string(second_number_[idx2]), cv::Point(200, 670), 
                cv::FONT_HERSHEY_DUPLEX, 1.75, 
                font_color, thickness, false);

        int time_p = (int)difftime(time(0), this->start_time);
        
        // char k = cv::waitKey(2);
        // if (k == 32) {
        if ((time_p % 6) == 0) {
            // ROS_INFO("***************Pressed***************");
            idx1 = (std::rand() % (int(first_number_.size())));
            idx2 = (std::rand() % (int(second_number_.size())));
        }
    }

    void drawRobotFootprint(){
        
        // temp:
        cv::Point p1(650, 800), p2(1050, 800), p3(800, 600), p4(900, 600);
        int thickness = 3;

        cv::line(this->fwd_img_ptr->image, 
                 p1, p3, cv::Scalar(255, 0, 0),
                 thickness, cv::LINE_8);

        cv::line(this->fwd_img_ptr->image, 
                 p2, p4, cv::Scalar(255, 0, 0),
                 thickness, cv::LINE_8);

        cv::line(this->fwd_img_ptr->image, 
                 p3, p4, cv::Scalar(255, 0, 0),
                 thickness, cv::LINE_8);
    }

    void displayVisual() 
    {
        // retrieve the start_timer & rear view bool state
        nh_.getParam("/start_timer", this->start_timer);
        nh_.getParam("toggle_rear_camera", this->show_rearview);
        nh_.getParam("debug_bars", this->show_debug_bars);

        if (this->fwd_img_ptr && this->bwd_img_ptr) {

            // draw top down map
            // this->drawTopViewMap();

            // draw robot boundaries
            this->drawRobotFootprint();

            // draw the scenario title
            // this->drawScenarioTitle();

            // draw the optimal speedbars
            if (this->show_debug_bars)
            // if (this->trial_condition == "HV-B" || this->trial_condition == "V-B")
                this->drawOptimalSpeedBars();

            // draw the operatpor speedbars
            this->drawOperatorSpeedBars();

            // draw indicator panels
            this->drawSideIndicators();

            // draw predicted trajectories
            if (this->trial_condition == "V-T" || this->trial_condition == "HV-T" || 
                this->trial_condition == "V-B" || this->trial_condition == "HV-B" )
            {
                this->drawPredictedTrajectories();
            }
                
            // draw force debug
            // this->drawForceBars();

            // draw the rearview camera 
            if (this->show_rearview) this->drawRearView();

            // check if simulation has started
            if (this->start_timer) {
                // start the timer ~ run this once
                for (static bool first = true; first; first=false) { this->start_time = time(0); }

                // if (this->task_title == "goal_aligned")
                //     this->drawTimer();
                 this->drawTimer();
            }
            
            // ROS_INFO("[width, height]: [%d, %d]", this->fwd_img_ptr->image.size().width, this->fwd_img_ptr->image.size().height);

            // if (this->distracted_mode)
            //     this->drawSecondaryTask();

            // Update GUI Window
            cv::imshow(window_name, this->fwd_img_ptr->image);
            cv::waitKey(2); // previously set to 5
            // cv::waitKey(1);

        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visual_interface");
    VisualInterface intf;
    // Proxemics prox();

    while (ros::ok())
    {
        intf.displayVisual();
        ros::spinOnce();
    }

    // ros::spin();
    
    return 0;
}