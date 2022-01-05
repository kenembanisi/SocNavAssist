#include <sarvo_local_planner/sarvo_local_planner.h>

const double PI = 3.14159;
const double INTIMATE_SPACE = 0.45;
const double PERSONAL_SPACE = 0.9;

namespace sarvo_local_planner {


// SARVOLocalPlanner::SARVOLocalPlanner(tf::TransformListener& tf) : tf_listener_(tf)
// SARVOLocalPlanner::SARVOLocalPlanner(tf2_ros::Buffer& tf, costmap_2d::Costmap2DROS* costmap_2d) : tf_(tf), costmap_ros_(costmap_2d)
SARVOLocalPlanner::SARVOLocalPlanner(tf2_ros::Buffer& tf) : tf_(tf)
// SARVOLocalPlanner::SARVOLocalPlanner()
// SARVOLocalPlanner::SARVOLocalPlanner(tf::TransformListener* tf, costmap_2d::Costmap2DROS costmap_2d) 
//     : tf_(tf), costmap_ros_(costmap_2d)
{
    /* initialize a pointer to a costmap */
    costmap_ros_ = new costmap_2d::Costmap2DROS("static_costmap", tf_);
    costmap_ros_->start();

    /* Get parameters from Parameter server */
    double goal_x, goal_y;
    nh_.param("scenario_name", scenario_, std::string("crossing"));
    nh_.param("radius_extension", radius_ext_, 0.2f);
    nh_.param("rvo_planning_horizon", rvo_planning_horizon_, 3.5f);
    nh_.param("goal_x", goal_x, 0.0);
    nh_.param("goal_y", goal_y, 0.0);
    nh_.param("rvo_alpha", alpha_, 1.0f);
    nh_.param("trial_condition", trial_condition_, std::string("auto"));
    
    nh_.param("/base_controller/linear/x/max_velocity", max_linear_vel_, 2.0);
    nh_.param("/base_controller/linear/x/max_acceleration", max_linear_acc_, 2.0);
    nh_.param("/base_controller/angular/x/max_velocity", max_angular_vel_, 2.0);
    nh_.param("/base_controller/angular/x/max_acceleration", max_angular_acc_, 2.0);

    nh_.getParam("/sarvo_planner/sarvo_local_planner/objective_name", objective_name_);
    nh_.getParam("/sarvo_planner/sarvo_local_planner/objective_weights_cautious", cautious_);
    nh_.getParam("/sarvo_planner/sarvo_local_planner/objective_weights_neutral", neutral_);
    nh_.getParam("/sarvo_planner/sarvo_local_planner/objective_weights_assertive", assertive_);
    
    /* Transform goal location [Temporary] */
    goal_location_.x = goal_y + 6.80;
    goal_location_.y = -goal_x - 5.52;

    /* Get frame information */
    costmap_frame_ = costmap_ros_->getGlobalFrameID();
    robot_base_frame_ = costmap_ros_->getBaseFrameID();


    ROS_INFO("SA-RVO Planner is using the following:");
    ROS_INFO("Costmap-Frame: [%s], Robot-Frame: [%s], Costmap Name: [%s]", 
        costmap_frame_.c_str(), robot_base_frame_.c_str(), costmap_ros_->getName().c_str());
    ROS_INFO("Scenario Name: [%s], Trial Condition: [%s]", 
        scenario_.c_str(), trial_condition_.c_str());


    /* Set the subscribers and publishers */
    std::string people_topic, groups_topic, odom_topic, cmd_topic;
    nh_.param("people_topic", people_topic, std::string("/spencer/perception/tracked_persons"));
    nh_.param("groups_topic", groups_topic, std::string("/spencer/perception/tracked_groups"));
    nh_.param("odom_topic", odom_topic, std::string("/base_controller/odom"));
    nh_.param("cmd_topic", cmd_topic, std::string("/base_controller/cmd_vel"));
    persons_subs_ = nh_.subscribe(people_topic, 10, &SARVOLocalPlanner::callbackTrackedPersons, this);
	groups_subs_ = nh_.subscribe(groups_topic, 10, &SARVOLocalPlanner::callbackTrackedGroups, this);
	odom_subs_ = nh_.subscribe(odom_topic, 10, &SARVOLocalPlanner::callbackWheelOdom, this);
	cmd_vel_subs_ = nh_.subscribe(cmd_topic, 10, &SARVOLocalPlanner::callbackCmdVel, this);
    suitable_traj_cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/suitable_trajectory_cloud", 1);
    unsuitable_traj_cloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/unsuitable_trajectory_cloud", 1);
    ped_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/pedestrian_pose_markers", 1);
    ped_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/pedestrian_viz_markers", 1);
    suitable_velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/suitable_velocities", 5);
    unsuitable_velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/unsuitable_velocities", 5);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic, 1);

    state_subs_ = nh_.subscribe("/gazebo/model_states", 10, &SARVOLocalPlanner::callbackStates, this);


    /* Set transform from world (Gazebo) to map frames */
    // tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -1.5707);
    // tf::Quaternion q = tf::createQuaternionFromYaw(-1.5707);
    // gazebo_to_map_transform_ = tf::Transform(q, tf::Vector3(-5.52, -6.80, 0.0));

    /* Initialize trajectory generator */
    double sim_time = 2.0;
    double vx_samples = 12;
    double vth_samples = 15;
    traj_generator_ = new TrajectoryGenerator(max_linear_vel_,
        max_angular_vel_, max_linear_acc_, max_angular_acc_,
        sim_time, vx_samples, vth_samples);  

    /* Initialize trajectory critic */  
    double horizon = sim_time;
    double clearance_threshold = 6.0;
    double sim_granularity = 0.2;
    std::vector<double> weights = selectWeights();
    traj_critic_ = new TrajectoryCritic(costmap_ros_->getCostmap(), weights, 
        horizon, clearance_threshold, sim_granularity);

    /* Initialize previous optimal point velocity */
    prev_v_optimal_.x = 0.0;
    prev_v_optimal_.y = 0.0;

}


bool SARVOLocalPlanner::isGoalReached()
{
    double dist = std::hypot(goal_location_.x - robot_.pose.x, 
                            goal_location_.y - robot_.pose.y);
    if (dist < goal_threshold_) return true;
    else return false;
}


void SARVOLocalPlanner::callbackWheelOdom(const nav_msgs::OdometryConstPtr& msg) 
{
    current_robot_vel_.vx = msg->twist.twist.linear.x;
    current_robot_vel_.vy = msg->twist.twist.linear.y;
    current_robot_vel_.w = msg->twist.twist.angular.z;

    generateAndPublishRobotCommand(); 
}


void SARVOLocalPlanner::callbackCmdVel(const geometry_msgs::TwistConstPtr& msg) 
{
    // update operator twist 
    operator_twist_.vx = msg->linear.x;
    operator_twist_.vy = msg->linear.y;
    operator_twist_.w = msg->angular.z;

    // compute operator vel
    operator_vel_ = twistToPoint2D(operator_twist_, robot_.pose.theta);

    // ROS_INFO("Twist: [%f, %f], PointVel: [%f, %f]",
    //     operator_twist_.vx, operator_twist_.w, operator_vel_.x, operator_vel_.y);

}


void SARVOLocalPlanner::callbackTrackedPersons(const spencer_tracking_msgs::TrackedPersonsConstPtr& msg) 
{
    tracked_persons_msg_ = (*msg);
}


void SARVOLocalPlanner::callbackTrackedGroups(const spencer_tracking_msgs::TrackedGroupsConstPtr& msg) {
    tracked_groups_msg_ = (*msg);
}


void SARVOLocalPlanner::callbackStates(const gazebo_msgs::ModelStatesConstPtr& msg)
{   
    std::string prefix;
    int agent_index;
    
    for (int i = 0; i < msg->name.size(); i++) {
        prefix = msg->name[i].substr(0, 5);
        if (prefix == "trina") { agent_index = i; }
    }

    robot_gndtruth_.x = msg->pose[agent_index].position.y + 6.80;
    robot_gndtruth_.y = -msg->pose[agent_index].position.x - 5.52;
    tf::Quaternion q(
            msg->pose[agent_index].orientation.x,
            msg->pose[agent_index].orientation.y,
            msg->pose[agent_index].orientation.z,
            msg->pose[agent_index].orientation.w  );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // robot_gndtruth_.theta = yaw - 1.5707;

    double theta;
    if (yaw >= -PI/2 && yaw <= PI) theta = yaw - PI/2;
    else theta = 1.5*PI + yaw;

    robot_gndtruth_.theta = theta;

    robot_gndtruth_.x += std::cos(robot_gndtruth_.theta)*radius_ext_;
    robot_gndtruth_.y += std::sin(robot_gndtruth_.theta)*radius_ext_;

}


void SARVOLocalPlanner::generateAndPublishRobotCommand()
{
    /* Update robot state */
    updateRobotState();

    /* Update pedestrian list */
    updatePedestrianList();

    /* Compute agent reference velocity*/
    goal_vel_ = computeGoalVelocity();

    /*
    if (isFinalGoalReached())
        // publish zero velocity to robot
    else if (isWayPointReached(curr_wp) && !path_queue.empty())
        curr_wp = path_queue.top();
        path_queue.pop();
    else if (!isWayPointVisible(curr_wp))
        curr_wp, path_queue = computePathtoGoal();

    */

    // ROS_INFO("Goal: [%f, %f]", goal_location_.x, goal_location_.y);
    // ROS_INFO("PointVel: [%f, %f]", vel.x, vel.y);

    /* Compute RVOs for all pedestrians */
    velocityObstacles_.clear();
    velocityObstacles_.reserve(ped_groups_.size());
    VelocityObstacle velocityObstacle;

    /* Define the desired velocity */
    Point2D v_desired;
    // v_desired.x = operator_vel_.x;
    // v_desired.y = operator_vel_.y;
    v_desired = goal_vel_;

    for (auto& person : ped_groups_){
        
        // distance btw agent and obstacle
        double dist = abs(person.pose, robot_.pose);
        double eff_obs_radius = person.radius + robot_.radius + eff_obs_radius_tol_;

        double dist_gnt_truth = abs(person.pose, robot_gndtruth_);

        // check that RVO is not computed for agent in collision with obs
        // if (eff_obs_radius > dist) dist = eff_obs_radius;
        if (eff_obs_radius > dist_gnt_truth) dist_gnt_truth = eff_obs_radius;

        // const double phi = std::asin(eff_obs_radius / dist); // phi is the angle btw vector connecting agent and obs and vector from 
                                                               // agent which is tangential with the obs effective boundary
        
        const double phi = std::asin(eff_obs_radius / dist_gnt_truth);
        
        // using RVO method, translate the apex of the RVO
        Point2D apex;
        // apex.x = robot_.pose.x + (1-alpha_) * robot_.velocity.x + alpha_ * person.velocity.x;
        // apex.y = robot_.pose.y + (1-alpha_) * robot_.velocity.y + alpha_ * person.velocity.y;
        apex.x = robot_.pose.x + (1-alpha_) * v_desired.x + alpha_ * person.velocity.x;
        apex.y = robot_.pose.y + (1-alpha_) * v_desired.y + alpha_ * person.velocity.y;
        // double theta = atan(person.pose, robot_.pose);

        double theta = atan(person.pose, robot_gndtruth_);

        // compute lambda_left and lambda right
        velocityObstacle.lambda_right_.x = std::cos(theta - phi);
        velocityObstacle.lambda_right_.y = std::sin(theta - phi);
        velocityObstacle.lambda_left_.x = std::cos(theta + phi);
        velocityObstacle.lambda_left_.y = std::sin(theta + phi);

        // velocityObstacle.lambda_right_.y = -std::cos(theta - phi);
        // velocityObstacle.lambda_right_.x = std::sin(theta - phi);
        // velocityObstacle.lambda_left_.y = -std::cos(theta + phi);
        // velocityObstacle.lambda_left_.x = std::sin(theta + phi);

        velocityObstacle.apex_ = apex;
        velocityObstacle.distance_ = dist;
        velocityObstacle.eff_obs_radius_ = eff_obs_radius;

        velocityObstacles_.push_back(velocityObstacle);



        // ROS_INFO_THROTTLE(2, "Lambda left [%f, %f], Lambda right [%f, %f], phi [%f], theta [%f]", 
        //     velocityObstacle.lambda_right_.x, velocityObstacle.lambda_right_.y,
        //     velocityObstacle.lambda_left_.x, velocityObstacle.lambda_left_.y,
        //     phi, theta);

        // ROS_INFO_THROTTLE(2, "CPP: phi [%f], theta [%f]",
        //     phi, theta);
        // ROS_INFO_THROTTLE(2, "CPP: dist_BA [%f], dist_GT [%f]",
        //     dist, dist_gnt_truth);

    }

    /* Compute suitable and unsuitable velocity candidates */
    std::vector<Candidate> v_suitable, v_unsuitable;
    checkIntersection(v_suitable, v_unsuitable);

    /* Compute the optimal 2D velocity */
    Candidate candidate_optimal = chooseOptimalVelocity(v_suitable, v_unsuitable);

    ROS_INFO("Optimal velocity: Point[%0.2f,%0.2f] Twist[%0.2f,%0.2f]", 
        candidate_optimal.velocity.x, candidate_optimal.velocity.y,
        candidate_optimal.twist.vx, candidate_optimal.twist.w);

    /* Publish command to robot*/
    geometry_msgs::Twist optimal_twist;
    optimal_twist.linear.x = candidate_optimal.twist.vx;
    optimal_twist.angular.z = candidate_optimal.twist.w;
    cmd_vel_pub_.publish(optimal_twist);

}


void SARVOLocalPlanner::updateRobotState()
{
    // tf::StampedTransform transform;
    // try {
    //     tf_listener_.waitForTransform(costmap_frame_,
    //         robot_base_frame_,
    //         ros::Time::now(),
    //         ros::Duration(0.05));
    //     tf_listener_.lookupTransform(costmap_frame_,
    //         robot_base_frame_,
    //         ros::Time::now(), transform);
    // }
    // catch (tf::TransformException& e) {
    //     ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from robot_base_frame to global_Frame failed. Reason: " << e.what());
    //     return;
    // }

    //  tf::Quaternion orientation(transform.getRotation().getX(),
    //     transform.getRotation().getY(),
    //     transform.getRotation().getZ(),
    //     transform.getRotation().getW());

    // double yaw = tf::getYaw(orientation);

    // /*----------------------------------------- */
    // Pose2D robot_pose_tmp;
    // robot_pose_tmp.x = transform.getOrigin().x();
    // robot_pose_tmp.y = transform.getOrigin().y();
    // robot_pose_tmp.z = transform.getOrigin().z();
    // robot_pose_tmp.theta = yaw;

    geometry_msgs::PoseStamped robot_global_pose;
    costmap_ros_->getRobotPose(robot_global_pose);

     tf::Quaternion orientation(robot_global_pose.pose.orientation.x,
        robot_global_pose.pose.orientation.y,
        robot_global_pose.pose.orientation.z,
        robot_global_pose.pose.orientation.w);

    double yaw = tf::getYaw(orientation);

    Pose2D robot_pose_tmp;
    robot_pose_tmp.x = robot_global_pose.pose.position.x;
    robot_pose_tmp.y = robot_global_pose.pose.position.y;
    robot_pose_tmp.z = robot_global_pose.pose.position.z;
    robot_pose_tmp.theta = yaw;

    // ROS_INFO("Robot pose is x: [%f], y: [%f], theta: [%f]",
    //     robot_global_pose.pose.position.x, 
    //     robot_global_pose.pose.position.y,

    // ROS_INFO("Robot pose is x: [%f], y: [%f], theta: [%f]",
    //     robot_pose_tmp.x, robot_pose_tmp.y, robot_pose_tmp.theta);

    /*----------------------------------------- */
    robot_.pose = augmentRobotPose(robot_pose_tmp);


    // ROS_INFO("Augmented robot pose is x: [%f], y: [%f], theta: [%f]",
    //     robot_.pose.x, robot_.pose.y, robot_.pose.theta);

    /*----------------------------------------- */
    robot_.radius = 0.25 + radius_ext_;

}


void SARVOLocalPlanner::updatePedestrianList()
{
    pedestrians_.clear();
    groups_.clear();
    ped_groups_.clear();
    // pedestrians_map_.clear();

    for (const spencer_tracking_msgs::TrackedPerson& p : tracked_persons_msg_.tracks){   
               
        // tf::Pose source;
        // double curr_or = tf::getYaw(p.pose.pose.orientation);
        // tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, curr_or);
        // tf::Matrix3x3 base(q);
        // source.setOrigin(tf::Vector3(p.pose.pose.position.x, p.pose.pose.position.x, 0.0));
        // source.setBasis(base);

        // tf::Pose result = gazebo_to_map_transform_ * source;
        // tf::Quaternion new_or = result.getRotation();
        // double theta = tf::getYaw(new_or);
        // double speed = std::hypot(p.twist.twist.linear.x, p.twist.twist.linear.y);
        
        // Person pd;
        // pd.pose.x = result.getOrigin().x();
        // pd.pose.y = result.getOrigin().y();
        // pd.pose.theta = theta;
        // pd.velocity.x = speed*cos(theta);
        // pd.velocity.y = speed*sin(theta);
        // pd.radius = 0.4;

        // -------------------------------------------------------------------------------
        // ROS_INFO("Gazebo frame-> Position: [%f, %f, %f]",
        //     p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z);

        // ROS_INFO("Map frame-> Position: [%f, %f]", x_pos, y_pos);
        // -------------------------------------------------------------------------------
        
        /*
        [Xmap] = [0  1] * [Xgazebo] + [ 6.80]
        [Ymap]   [-1 0]   [Ygazebo]   [-5.52]
        */
        Person pd;
        pd.type = "single";
        pd.pose.x = p.pose.pose.position.y + 6.80;
        pd.pose.y = -p.pose.pose.position.x - 5.52;
        pd.pose.theta = 0.0;
        pd.velocity.x = p.twist.twist.linear.y;
        pd.velocity.y = -p.twist.twist.linear.x;
        pd.radius = PERSONAL_SPACE; //0.9
        pd.person_id = p.track_id;

        // pedestrians_map_.insert(std::make_pair(p.track_id, pd));
        pedestrians_.emplace_back(pd);
        ped_groups_.emplace_back(pd);

    
    }


    for (const spencer_tracking_msgs::TrackedGroup& g : tracked_groups_msg_.groups){
    //     tf::Pose source;
    //     double curr_or = tf::getYaw(g.centerOfGravity.pose.orientation);
    //     tf::Quaternion q = tf::createQuaternionFromYaw(curr_or);
    //     tf::Matrix3x3 base(q);
    //     source.setOrigin(tf::Vector3(g.centerOfGravity.pose.position.x, g.centerOfGravity.pose.position.y, 0.0));
    //     source.setBasis(base);

    //     tf::Pose result = gazebo_to_map_transform_ * source;
    //     tf::Quaternion new_or = result.getRotation();
    //     double theta = tf::getYaw(new_or);

        Person gr;
        gr.type = "group";
        gr.pose.x = g.centerOfGravity.pose.position.y + 6.80;
        gr.pose.y = -g.centerOfGravity.pose.position.x - 5.52;
        gr.pose.theta = 0.0;
        // gr.person_ids = g.track_ids;

        // compute group velocity as the average of the pedestrians
        Point2D velocity;
        for (auto id : g.track_ids){
            velocity.x += pedestrians_[id].velocity.x;
            velocity.y += pedestrians_[id].velocity.y;
        }
        gr.velocity.x = velocity.x/g.track_ids.size();
        gr.velocity.y = velocity.y/g.track_ids.size();

        // compute group radius
        double radius = 0.0, ped_to_center_distance;
        for (auto id : g.track_ids){
            ped_to_center_distance = std::hypot((pedestrians_[id].pose.x - gr.pose.x),
                                                (pedestrians_[id].pose.y - gr.pose.y));
            radius = (ped_to_center_distance > radius) ? ped_to_center_distance : radius;
        }
        gr.radius = radius;

        // pedestrians_map_.insert(std::make_pair(g.group_id, gr));  // this might be confusing later for checking track_ids
        groups_.emplace_back(gr);
        ped_groups_.emplace_back(gr);
    }

    // -------------------------------------------------------------------------------
    // for (auto& p : ped_groups_){
    //     if (p.type == "single") {
    //         ROS_INFO("Person: [%d], is at position: [%f, %f]",
    //             (int)p.person_id, p.pose.x, p.pose.y);
    //     }
    //     else {
    //         ROS_INFO("Group: is at position: [%f, %f]",
    //             p.pose.x, p.pose.y);
    //     }
    // }
    // -------------------------------------------------------------------------------
}


Pose2D SARVOLocalPlanner::augmentRobotPose(Pose2D& pose)
{
    Pose2D augmented_pose;
    augmented_pose.x = pose.x + std::cos(pose.theta)*radius_ext_;
    augmented_pose.y = pose.y + std::sin(pose.theta)*radius_ext_;
    augmented_pose.theta = pose.theta;
    return augmented_pose;
}


Point2D SARVOLocalPlanner::vectorTransform(Point2D& vec, double theta)
{
    Point2D result;
    result.x = vec.x*std::cos(theta) - vec.y*std::sin(theta);
    result.y = vec.x*std::sin(theta) + vec.y*std::cos(theta);
    return result;
}


Point2D SARVOLocalPlanner::twistToPoint2D(Twist2D& twist, double theta)
{
    Point2D result;

    // if (PI/2 < theta < PI || -PI <= theta < -PI/2){
    //     // twist.w = -1 * twist.w;
    //     result.x = twist.vx * std::cos(theta) + radius_ext_ * twist.w * std::sin(theta);
    //     result.y = twist.vx * std::sin(theta) - radius_ext_ * twist.w * std::cos(theta);
    // } 
    // else {
    //     result.x = twist.vx * std::cos(theta) - radius_ext_ * twist.w * std::sin(theta);
    //     result.y = twist.vx * std::sin(theta) + radius_ext_ * twist.w * std::cos(theta);
    // }

    if (twist.vx < 0){
        // twist.w = -1 * twist.w;
        result.x = twist.vx * std::cos(theta) + radius_ext_ * twist.w * std::sin(theta);
        result.y = twist.vx * std::sin(theta) - radius_ext_ * twist.w * std::cos(theta);
    } 
    else {
        result.x = twist.vx * std::cos(theta) - radius_ext_ * twist.w * std::sin(theta);
        result.y = twist.vx * std::sin(theta) + radius_ext_ * twist.w * std::cos(theta);
    }

    // result.x = twist.vx * std::cos(theta) - radius_ext_ * twist.w * std::sin(theta);
    // result.y = twist.vx * std::sin(theta) + radius_ext_ * twist.w * std::cos(theta);
    return result;
}


Point2D SARVOLocalPlanner::computeGoalVelocity()
{
    Point2D goal_vel, vec;
    // find the vector pointing from agent pose to goal and compute magnitute of vector
    double mag = std::hypot(goal_location_.x - robot_.pose.x, 
                            goal_location_.y - robot_.pose.y);
    vec.x = (goal_location_.x - robot_.pose.x) * max_linear_vel_ / mag;
    vec.y = (goal_location_.y - robot_.pose.y) * max_linear_vel_ / mag;

    // check if the agent has reached the goal
    if (isGoalReached()) {
        vec.x = 0.0; vec.y = 0.0;
    }
    return vec;
}


void SARVOLocalPlanner::checkIntersection(std::vector<Candidate>& vSuitable, 
    std::vector<Candidate>& vUnsuitable)
{
    traj_generator_->startNewIteration(robot_.pose,
        goal_location_, operator_twist_, 0.5);

    // for trajectory visualization
    std::vector<Trajectory2D> suitable_trajectories;
    std::vector<Trajectory2D> unsuitable_trajectories;


    // ROS_INFO("Theta: [%f, %f]", robot_.pose.theta, robot_gndtruth_.theta);

    while (traj_generator_->hasMoreSamples())
    {
        Candidate candidate;
        Twist2D next_twist = traj_generator_->nextVelocity();
        // candidate.velocity = twistToPoint2D(next_twist, robot_.pose.theta);

        // candidate.
        candidate.twist = next_twist;
        candidate.velocity = twistToPoint2D(next_twist, robot_gndtruth_.theta);

        
        
        // Generate a new trajectory
        // candidate.traj = traj_generator_->generateTrajectory(robot_.pose, 
        //     operator_twist_, next_twist);
        candidate.traj = traj_generator_->generateTrajectory(robot_gndtruth_, 
            operator_twist_, next_twist);


        // Check for static collision
        traj_critic_->computeCandidateScore(candidate, "obstacle");
        if (candidate.score.raw_scores[0] > 250) continue; // i.e. skip the candidate

        // Check if the candidate velocity is suitable
        bool suitable = true;
        for (VelocityObstacle& vo : velocityObstacles_)
        {
            Point2D relative_vel;
            relative_vel.x = candidate.velocity.x + robot_.pose.x - vo.apex_.x;
            relative_vel.y = candidate.velocity.y + robot_.pose.y - vo.apex_.y;
            // find the angles the RVO boundaries make with the global X and then check if 
            // the angle vAB makes with the global X is within that
            double relative_vel_theta, theta_right, theta_left;
            // relative_vel_theta = std::atan2(relative_vel.y, relative_vel.x);
            // theta_left = std::atan2(vo.lambda_left_.y, vo.lambda_left_.x);
            // theta_right = std::atan2(vo.lambda_right_.y, vo.lambda_right_.x);

            relative_vel_theta = atan2m(relative_vel.y, relative_vel.x);
            theta_left = std::atan2(vo.lambda_left_.y, vo.lambda_left_.x);
            theta_right = std::atan2(vo.lambda_right_.y, vo.lambda_right_.x);

            

            // ROS_INFO("candidate: [%f, %f]",
            //     candidate.velocity.x, candidate.velocity.y );
            
            // ROS_INFO_THROTTLE(2, "Theta left/right [%f, %f], Theta_v [%f]" ,
            // theta_left, theta_right, relative_vel_theta);



            // check if the velocity vector is suitable by:
                // (1) checking if theta_relative_vel falls between theta_right and theta_left
            // if (inBetween(relative_vel_theta, theta_left, theta_right) &&
            //     imminentCollision(relative_vel, vo.distance_, vo.eff_obs_radius_)) {
            //     suitable = false;
            //     break; }
            if (inBetween(relative_vel_theta, theta_left, theta_right) &&
                imminentCollision(relative_vel, vo.distance_, vo.eff_obs_radius_)) {
                suitable = false;
                break; 
            }

            // ROS_INFO("Vel:[%0.2f,%0.2f], Tw:[%0.2f,%0.2f], rel_th[%0.3f], Th l/r[%0.2f,%0.2f], Rob Th[%0.2f, %0.2f], suit[%d]", 
            //     candidate.velocity.x, candidate.velocity.y,
            //     next_twist.vx, next_twist.w,
            //     relative_vel_theta,
            //     theta_left, theta_right,
            //     robot_gndtruth_.theta,
            //     robot_.pose.theta,
            //     suitable);

        }
        
        if (suitable) {
            vSuitable.push_back(candidate);
            suitable_trajectories.push_back(candidate.traj);
        }
        else {
            vUnsuitable.push_back(candidate);
            unsuitable_trajectories.push_back(candidate.traj);
        }

        


    }

    ROS_INFO("/////////////////////////");

    // ROS_INFO("Suitable: [%d] | Unsuitable: %d]", (int)vSuitable.size(), (int)vUnsuitable.size());


    trajectoryCloudPublisher(suitable_trajectories, unsuitable_trajectories);
    pedestrianPosePublisher();
    pointVelocityPublisher(vSuitable, vUnsuitable);

}


void SARVOLocalPlanner::trajectoryCloudPublisher(const std::vector<Trajectory2D>& suitable_traj_array,
	const std::vector<Trajectory2D>& unsuitable_traj_array)
{
    geometry_msgs::PoseArray suitable_traj_cloud;
    geometry_msgs::PoseArray unsuitable_traj_cloud;
    suitable_traj_cloud.header.frame_id = "map";

    for (auto& traj : suitable_traj_array){
        for (auto& p : traj.poses){
            geometry_msgs::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = p.z;
            tf::Quaternion q = tf::createQuaternionFromYaw(p.theta);
            pose.orientation.x = q.getX();
            pose.orientation.y = q.getY();
            pose.orientation.z = q.getZ();
            pose.orientation.w = q.getW();

            suitable_traj_cloud.poses.push_back(pose);
        }
    }

    unsuitable_traj_cloud.header.frame_id = "map";

    for (auto& traj : unsuitable_traj_array){
        for (auto& p : traj.poses){
            geometry_msgs::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = p.z;
            tf::Quaternion q = tf::createQuaternionFromYaw(p.theta);
            pose.orientation.x = q.getX();
            pose.orientation.y = q.getY();
            pose.orientation.z = q.getZ();
            pose.orientation.w = q.getW();

            unsuitable_traj_cloud.poses.push_back(pose);
        }
    }

    unsuitable_traj_cloud_pub_.publish(unsuitable_traj_cloud);
    suitable_traj_cloud_pub_.publish(suitable_traj_cloud);
}


void SARVOLocalPlanner::pointVelocityPublisher(const std::vector<Candidate>& suitable_candidate_vector,
    const std::vector<Candidate>& unsuitable_candidate_vector)
{
    visualization_msgs::MarkerArray suitable_velocity_arrows;
    int idx = 0;

    for (auto& v : suitable_candidate_vector)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        ros::Time time = ros::Time();
        marker.id = idx;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.04;
        marker.scale.y = 0.08;
        marker.scale.z = 0;
        marker.color.a = 0.9;
        marker.color.r = 0.25;
        marker.color.g = 0.4;
        marker.color.b = 0.75;
        geometry_msgs::Point start_point, end_point;
        start_point.x = robot_gndtruth_.x;
        start_point.y = robot_gndtruth_.y;
        end_point.x = robot_gndtruth_.x + v.velocity.x;
        end_point.y = robot_gndtruth_.y + v.velocity.y;
        
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        idx++;

        suitable_velocity_arrows.markers.push_back(marker);

        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "map";
        // ros::Time time = ros::Time();
        marker2.id = idx;
        marker2.type = visualization_msgs::Marker::ARROW;
        marker2.scale.x = 0.04;
        marker2.scale.y = 0.08;
        marker2.scale.z = 0;
        marker2.color.a = 0.6;
        marker2.color.r = 0.75;
        marker2.color.g = 0.4;
        marker2.color.b = 0.75;
        // geometry_msgs::Point start_point, end_point;
        start_point.x = robot_gndtruth_.x;
        start_point.y = robot_gndtruth_.y;
        end_point.x = robot_gndtruth_.x + goal_vel_.x;
        end_point.y = robot_gndtruth_.y + goal_vel_.y;

        marker2.points.push_back(start_point);
        marker2.points.push_back(end_point);

        idx++;

        suitable_velocity_arrows.markers.push_back(marker2);
        
    }

    visualization_msgs::MarkerArray unsuitable_velocity_arrows;
    idx = 0;
    for (auto& v : unsuitable_candidate_vector)
    {
        idx++;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        ros::Time time = ros::Time();
        marker.id = idx;
        marker.type = visualization_msgs::Marker::ARROW;
        // marker.pose.position.x = p.pose.x;
        // marker.pose.position.y = p.pose.y;
        // marker.pose.position.z = 0.0;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.04;
        marker.scale.y = 0.08;
        marker.scale.z = 0;
        marker.color.a = 0.9;
        marker.color.r = 0.25;
        marker.color.g = 0.6;
        marker.color.b = 0.25;
        marker.lifetime = ros::Duration(0.1);
        geometry_msgs::Point start_point, end_point;
        start_point.x = robot_gndtruth_.x;
        start_point.y = robot_gndtruth_.y;
        end_point.x = robot_gndtruth_.x + v.velocity.x;
        end_point.y = robot_gndtruth_.y + v.velocity.y;
        
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        unsuitable_velocity_arrows.markers.push_back(marker);
    }

    suitable_velocity_pub_.publish(suitable_velocity_arrows);
    unsuitable_velocity_pub_.publish(unsuitable_velocity_arrows);

}


void SARVOLocalPlanner::pedestrianPosePublisher()
{
    geometry_msgs::PoseArray pedestrian_markers;
    pedestrian_markers.header.frame_id = "map";

    visualization_msgs::MarkerArray pedestrian_viz_markers;
    int idx = 0;

    for (auto& p : ped_groups_){
        // for pose array
        geometry_msgs::Pose pose;
        pose.position.x = p.pose.x;
        pose.position.y = p.pose.y;
        pose.position.z = 1.0;
        tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();

        pedestrian_markers.poses.push_back(pose);
        
        // for marker viz array
        idx++;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        ros::Time time = ros::Time();
        marker.id = idx;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = p.pose.x;
        marker.pose.position.y = p.pose.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = PERSONAL_SPACE*2;
        marker.scale.y = PERSONAL_SPACE*2;
        marker.scale.z = 0.01;
        marker.color.a = 0.5;
        marker.color.r = 0.25;
        marker.color.g = 0.4;
        marker.color.b = 0.25;

        pedestrian_viz_markers.markers.push_back(marker);

        idx++;
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "map";
        // ros::Time time = ros::Time();
        marker2.id = idx;
        marker2.type = visualization_msgs::Marker::CYLINDER;
        marker2.pose.position.x = p.pose.x;
        marker2.pose.position.y = p.pose.y;
        marker2.pose.position.z = 0.0;
        marker2.pose.orientation.x = 0.0;
        marker2.pose.orientation.y = 0.0;
        marker2.pose.orientation.z = 0.0;
        marker2.pose.orientation.w = 1.0;
        marker2.scale.x = PERSONAL_SPACE*2 + robot_.radius*2;
        marker2.scale.y = PERSONAL_SPACE*2 + robot_.radius*2;
        marker2.scale.z = 0.01;
        marker2.color.a = 0.3;
        marker2.color.r = 0.2;
        marker2.color.g = 0.25;
        marker2.color.b = 0.2;

        pedestrian_viz_markers.markers.push_back(marker2);
    }


    ped_pose_pub_.publish(pedestrian_markers);
    ped_viz_pub_.publish(pedestrian_viz_markers);
}


std::vector<double> SARVOLocalPlanner::selectWeights()
{
    std::vector<double> weights = {0.1, 0.1, 0.1, 0.1, 0.1};

    if (objective_name_ == "cautious") 
        weights = cautious_;
    else if (objective_name_ == "neutral") 
        weights = neutral_;
    else if (objective_name_ == "assertive") 
        weights = assertive_;
    else {
        ROS_ERROR("Invalid objective/behavior name [%s]", objective_name_.c_str());
    }
    return weights;
}


bool SARVOLocalPlanner::inBetween(double theta_v,
    double theta_left, double theta_right)
{
    // check if RVO angle < PI
    if (std::abs(theta_right - theta_left) <= PI){ 
        if (theta_right <= theta_v && theta_v <= theta_left) return true;
        else return false;
    }
    else {
        if (theta_left < 0 && theta_right > 0){
            theta_left += 2*PI;
            if (theta_v < 0) theta_v += 2*PI;
            // if (theta_right <= theta_v <= theta_left) 
            if (theta_v >= theta_right && theta_v <= theta_left) return true;
            // {
            //     ROS_INFO("True----------Point 2----------: Th l/r[%0.2f,%0.2f] rel_th[%0.2f]", theta_left, theta_right, theta_v);
            //     return true;
            // }
            else return false;
        }
        if (theta_left > 0 && theta_right < 0){
            theta_right += 2*PI;
            if (theta_v < 0) theta_v += 2*PI;
            // if (theta_left <= theta_v <= theta_right) return true;
            if (theta_v <= theta_right && theta_v >= theta_left > 0) return true;
            else return false;
        }
    }
}


bool SARVOLocalPlanner::imminentCollision(const Point2D& vel,
    const double distance, const double radius)
{
    double vel_mag = std::hypot(vel.x, vel.y);
    // calculate the min vel for an imminent collision
    double min_vel_imminent = (distance - radius) / rvo_planning_horizon_;

    // check if candidate velocity is greater than the minimum collision-imminent velocity
    return vel_mag >= min_vel_imminent;
}


Candidate SARVOLocalPlanner::chooseOptimalVelocity(std::vector<Candidate>& v_suitable, 
	std::vector<Candidate>& v_unsuitable)
{
    Candidate optimal_candidate;
    optimal_candidate.score.total = std::numeric_limits<double>::max();

    if (v_suitable.size() > 0)
    {
        for (auto& candidate : v_suitable)
        {
            // Compute the score of the candidate
            // traj_critic_->computeCandidateScore(candidate, prev_v_optimal_);

            // Feature 2: Change from previous optimal velocity
            candidate.score.raw_scores[1] = abs(candidate.velocity, prev_v_optimal_);
            // Feature 3: Deviation from operator's input
            candidate.score.raw_scores[2] = abs(candidate.velocity, operator_vel_);
            // Feature 4: Deviation from goal heading
            candidate.score.raw_scores[3] = abs(candidate.velocity, goal_vel_);
            // Feature 5: Social obstruction score
            candidate.score.raw_scores[4] = 
                traj_critic_->socialDisturbanceScore(candidate, ped_groups_);

            // compute weighted cost
            // traj_critic_->computeTotalScore(candidate);

            candidate.score.total = abs(candidate.velocity, goal_vel_) + candidate.score.raw_scores[0];

            // if (std::abs(candidate.velocity.x + candidate.velocity.y) < 0.3) candidate.score.total += 1.5;

            // update the best candidate based on score
            // if (optimal_candidate.score.total < 0 ||
            //     candidate.score.total < optimal_candidate.score.total)
            //     optimal_candidate = candidate; // lower score is better
            if (candidate.score.total < optimal_candidate.score.total)
                optimal_candidate = candidate;
            
  
            ROS_INFO("Vel:[%0.2f,%0.2f], Tw:[%0.2f,%0.2f], score[%0.3f]", 
                candidate.velocity.x, candidate.velocity.y,
                candidate.twist.vx, candidate.twist.w,
                candidate.score.total);

        }
    }

    return optimal_candidate;
    
}


} // end namespace





int main(int argc, char** argv){

	ros::init(argc, argv, "sarvo_planner_node");

	// tf::TransformListener tf;
    // tf2_ros::Buffer buffer(ros::Duration(10));

    tf2_ros::Buffer tf;
    tf2_ros::TransformListener tfListener(tf);

    // tf::TransformListener tf(ros::Duration(10));
	// costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tf);

	// namespace::ClassName object_name
	// sarvo_local_planner::SARVOLocalPlanner sarvo_planner(tf, costmap_ros);
    sarvo_local_planner::SARVOLocalPlanner sarvo_planner(tf);

	ros::spin();

	return 0;

}