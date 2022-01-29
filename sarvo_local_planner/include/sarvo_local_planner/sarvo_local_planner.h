
// sarvo_local_planner.h

// ROS includes
#include <ros/ros.h>
#include "tf/transform_listener.h"
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <stack>
#include <unordered_map>
#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Candidate.h"
#include "sarvo_msgs/Person.h"
#include "sarvo_msgs/Twist2D.h"
#include "sarvo_msgs/SimulationStates.h"
#include "costmap_2d/costmap_2d.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedGroups.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <sarvo_local_planner/trajectory_generator.h>
#include <sarvo_local_planner/trajectory_critic.h>
#include <sarvo_local_planner/path_planner.h>


// Other includes
#include <cmath>

using namespace sarvo_msgs;

namespace sarvo_local_planner {
	
	class SARVOLocalPlanner {
		
		public:
			// methods
			
			/**
			 * @brief  Constructor
			 * @param  ~
			 */
			// SARVOLocalPlanner(tf::TransformListener tf, costmap_2d::Costmap2DROS* costmap_ros);
			// SARVOLocalPlanner(tf2_ros::Buffer& tf, costmap_2d::Costmap2DROS* costmap_ros);
			SARVOLocalPlanner(tf2_ros::Buffer& tf);
            SARVOLocalPlanner();

			// SARVOLocalPlanner(tf2_ros::Buffer& tf);
			
			/**
			 * @brief  Destructor
			 */
			~SARVOLocalPlanner();

		private:
			// methods:
			
			/**
			 * @brief  Runs SA-RVO to generate velocity and the publishes on cmd_vel to the robot
			 * @param  ~
			 */
			void generateAndPublishRobotCommand(); 

			/**
			 * @brief  Checks to see if the robot is in the goal location
			 * @return True if the robot is within the threshold of the goal location
			 */
			bool isFinalGoalReached();

			/**
			 * @brief  Checks to see if the robot is in the goal location
			 * @param  cmd_vel topic message
			 */
			void callbackCmdVel(const geometry_msgs::TwistConstPtr& msg); 

            /**
			 * @brief  
			 * @param  
			 */
			void callbackTrackedPersons(const spencer_tracking_msgs::TrackedPersonsConstPtr& msg); 

            /**
			 * @brief  
			 * @param  
			 */
			void callbackTrackedGroups(const spencer_tracking_msgs::TrackedGroupsConstPtr& msg); 

            /**
			 * @brief  
			 * @param  
			 */
			void callbackWheelOdom(const nav_msgs::OdometryConstPtr& msg); 

			/**
			 * @brief  
			 * @param  
			 */
			void callbackStates(const gazebo_msgs::ModelStatesConstPtr& msg);
			
            /**
			 * @brief  Updates both the pose and velocity of the robot
			 */
			void updateRobotState();

            /**
			 * @brief 
			 */
			void updatePedestrianList();
		
			/**
			 * @brief  Augments the pose of the robot with the additional radius
			 */
			Pose2D augmentRobotPose(Pose2D& pose);

			/**
			 * @brief  ~
			 */
			Point2D vectorTransform(Point2D& vec, double theta);

			/**
			 * @brief  ~
			 */
			Point2D twistToPoint2D(Twist2D& twist, double theta);

			/**
			 * @brief  ~
			 */
			Point2D computeGoalVelocity(Pose2D& goal);

			/**
			 * @brief  ~
			 * @param  ~
			 */
			void checkIntersection(std::vector<Candidate>& v_suitable, 
					std::vector<Candidate>& v_unsuitable);

			/**
			 * @brief  ~
			 * @param  ~
			 */
			sarvo_msgs::Candidate chooseOptimalVelocity(std::vector<Candidate>& v_suitable, 
					std::vector<Candidate>& v_unsuitable);

			/**
			 * @brief  ~
			 * @param  ~
			 */
			void updateCandidateScore(sarvo_msgs::Candidate& candidate, std::string& critic_name);


			/**
			 * @brief  ~
			 * @param  ~
			 */
			geometry_msgs::Twist getransformVelocity(Point2D v_optimal);

			
			void trajectoryCloudPublisher(const std::vector<Trajectory2D>& suitable_traj_array,
				const std::vector<Trajectory2D>& unsuitable_traj_array);


			void pointVelocityPublisher(const std::vector<Candidate>& suitable_candidate_vector,
    			const std::vector<Candidate>& unsuitable_candidate_vector);


			void pedestrianPosePublisher();


			void simulationStatesPublisher(const Candidate& candidate);


			void simulationStatesPublisher(const Candidate& candidate_optimal,
    			const Candidate& candidate_operator);


			std::vector<double> selectWeights();


			bool inBetween(double theta_v, double theta_left, 
				double theta_right);


			bool imminentCollision(const Point2D& vel, const double distance, 
				const double radius);


			sarvo_msgs::Candidate computeOperatorVelocityCost(const Point2D operator_vel_);


			void optimalTwistPublisher(const Candidate& optimal_candidate);

			void headingDeltaPublisher(const Point2D& optimal_vel,
    			const Point2D& operator_vel);


		private:
			// member variables:

            // set the ROS nodehandles
            // ros::NodeHandle private_nh("~");
            ros::NodeHandle nh_;
            tf::TransformListener tf_listener_;
			tf2_ros::Buffer& tf_;
			costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap

            // robot and pedestrian states
            Person robot_;
			Pose2D robot_gndtruth_;
            Twist2D current_robot_vel_;
            std::vector<Person> pedestrians_, groups_, ped_groups_;
			spencer_tracking_msgs::TrackedPersons tracked_persons_msg_;
			spencer_tracking_msgs::TrackedGroups tracked_groups_msg_;
			SimulationStates sim_states_;
            // std::map<int, Person> pedestrians_map_;
			// std::unset<Person> pedestrians_set_;

            // goal pose
            Pose2D goal_location_;
			Pose2D start_location_;
			Twist2D operator_twist_;
			Point2D operator_vel_;

            // costmap
            costmap_2d::Costmap2DROS* global_costmap_;
            std::string costmap_frame_;
            std::string robot_base_frame_;

            // parameters
            std::string scenario_;
			std::string feature_filename_;
			std::string b_weights_;
            float radius_ext_;
            float rvo_planning_horizon_;
            float alpha_;
            std::string trial_condition_;
			double max_linear_vel_;
			double max_angular_vel_;
			double max_linear_acc_;
			double max_angular_acc_;
			float goal_threshold_ = 0.8f;
			std::vector<double> cautious_;
			std::vector<double> neutral_;
			std::vector<double> assertive_;
			std::string objective_name_;
			std::vector<double> prm_samples_x_;
			std::vector<double> prm_samples_y_;
			// std::vector< std::vector<double> > prm_roadmap_;
			double robot_fov_;
			double config_space_step_size_;
			double connecting_node_dist_thr_;
			XmlRpc::XmlRpcValue prm_roadmap_;
            
            // subs and pubs
            ros::Subscriber persons_subs_;
            ros::Subscriber groups_subs_;
            ros::Subscriber odom_subs_;
            ros::Subscriber cmd_vel_subs_;
			ros::Publisher suitable_traj_cloud_pub_;
			ros::Publisher unsuitable_traj_cloud_pub_;
			ros::Publisher suitable_velocity_pub_;
			ros::Publisher unsuitable_velocity_pub_;
			ros::Publisher ped_pose_pub_;
			ros::Publisher ped_viz_pub_;
			ros::Publisher cmd_vel_pub_;
			ros::Publisher sim_states_pub_;
			ros::Publisher optimal_cmd_vel_pub_;
			ros::Publisher heading_delta_pub_;

			ros::Subscriber state_subs_;

			tf::Transform gazebo_to_map_transform_;

			std::vector<VelocityObstacle> velocityObstacles_;
			double eff_obs_radius_tol_ = 0.1;

			TrajectoryGenerator* traj_generator_;
			TrajectoryCritic* traj_critic_;
			PathPlanner* path_planner_;

			Point2D prev_v_optimal_;
			Point2D goal_vel_;
			std::vector<double> operator_feature_count_ 
				= std::vector<double>(5);
			
			geometry_msgs::Twist zero_twist_;
			bool isInitPathDefined_ = false;
			std::stack<Pose2D> path_to_goal_;
			Pose2D current_wp_;
    };

}