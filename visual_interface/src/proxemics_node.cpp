// includes
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float32.h>
#include <algorithm>
#include <tf/tf.h>
#include <cmath>


/*
    Agent pose struct
*/
struct Pose {
    std::string name = "";
    float x = 0;
    float y = 0;
    float theta = 0;

    // Constructors
    Pose() {}
    Pose(std::string &agent_name) : name(agent_name) {}

    // // Destructors
    // ~Pose() { delete; }
    // ~Pose() { delete[]; }

};

/*
    Proxemics class
*/

class Proxemics {

    //---------------------------------------------------------------------------------------------------
    // private data members
    private:
        // ROS objects
        ros::NodeHandle nh_;
        ros::Subscriber states_subcriber_;
        ros::Publisher proxemics_publisher_;
        ros::Publisher proxemics_score_publisher_;

        // variables
        std_msgs::Int8MultiArray proxemics_state_;
        std_msgs::Float32 proxemics_score_;
        std::vector<int> actor_index_;
        int num_actors_;
        int agent_index_;
        std::vector<Pose> actor_poses_;
        Pose agent_pose_;
        float agent_radius_ = 0.4f;
        float clearance_threshold_ = 1.75f;
        float space_radius_ = 0.45f + agent_radius_;
        int intrusion_ = 0;
        

    //---------------------------------------------------------------------------------------------------
    // public member functions
    public:

        /* Constructor */
        Proxemics(ros::NodeHandle& nh) : nh_(nh) {
            
            // initialize the subscriber
            states_subcriber_ = nh_.subscribe("/gazebo/model_states", 10, &Proxemics::statesCb, this);

            // initialize the publisher
            proxemics_publisher_ = nh_.advertise<std_msgs::Int8MultiArray>("/proxemics_states", 50);
            proxemics_score_publisher_ = nh_.advertise<std_msgs::Float32>("/proxemics_score", 50);
            
            // get index of actors and agent
            boost::shared_ptr<gazebo_msgs::ModelStates const> msg_ptr;
            msg_ptr = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
            std::string prefix;

            for (int i = 0; i < msg_ptr->name.size(); i++) {
                prefix = msg_ptr->name[i].substr(0, 5);
                if (prefix == "actor") { actor_index_.push_back(i); num_actors_ = actor_index_.size(); }
                if (prefix == "trina") { agent_index_ = i; }
            }
            
            ROS_INFO("Initialized the Proxemics Evaluation Node");

        }

        // Destructor
        ~Proxemics() {}

        
        // publisher
        void publishProxemics(void) {

            // set proxemics states
            proxemics_state_.data.clear();

            // check if actor_poses is populated
            if (actor_poses_.size() > 1) {
                checkIntrusion();
                // ROS_INFO("Intrusion checker: %d", intrusion_);
            }

            // update proxemics_state message
            proxemics_state_.data.push_back(intrusion_);

            // publish message
            proxemics_publisher_.publish(proxemics_state_);
            
        }

        void publishProxemicsScore(void) {
            
            // clear proxemics score
            proxemics_score_.data = 0.0;

            // check if actor_poses is populated
            if (actor_poses_.size() > 1) {
                proxemics_score_.data = calcScore();
                // ROS_INFO("Proxemics score: %0.3f", proxemics_score_.data);
            }

            // publish message
            proxemics_score_publisher_.publish(proxemics_score_);
        }



    //---------------------------------------------------------------------------------------------------
    // private member functions
    private:
        // subscriber call back
        void statesCb(const gazebo_msgs::ModelStates& msg) {

            // clear previous actors in vector
            actor_poses_.clear();

            // get list of model states
            for (int i : actor_index_) {
                Pose actor_pose;
                actor_pose.name = msg.name[i];
                actor_pose.x = msg.pose[i].position.x;
                actor_pose.y = msg.pose[i].position.y;
                actor_poses_.push_back(actor_pose);
            }

            agent_pose_.name = msg.name[agent_index_];
            agent_pose_.x = msg.pose[agent_index_].position.x;
            agent_pose_.y = msg.pose[agent_index_].position.y;
            tf::Quaternion q(
                    msg.pose[agent_index_].orientation.x,
                    msg.pose[agent_index_].orientation.y,
                    msg.pose[agent_index_].orientation.z,
                    msg.pose[agent_index_].orientation.w  );
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            agent_pose_.theta = yaw;


            publishProxemics();

            publishProxemicsScore();

        }

        void checkIntrusion(void) {
            for (int i = 0; i < num_actors_; i++) {
                float clearance = calcDistance(agent_pose_, actor_poses_[i]);
                if (clearance <= space_radius_) { intrusion_ = 1; break; } // not counting the number of collisions
                else { intrusion_ = 0; }
            }
        }

        float calcDistance(Pose &agent_pose, Pose &actor_pose) {
            float distance = sqrt( pow(agent_pose.x - actor_pose.x, 2) + 
                                   pow(agent_pose.y - actor_pose.y, 2));
            return distance;
        }

        float calcScore(void) {
            
            float score = 0.0;
            
            for (size_t i = 0; i < num_actors_; ++i) {
                float dist = calcDistance(agent_pose_, actor_poses_[i]);

                if (dist < clearance_threshold_) {
                    float ped_score = (clearance_threshold_ - dist);
                    score = std::max(score, ped_score);
                }
            }

            return score;
        }

};

int main(int argc, char** argv)
{
    // calling ROS:init
    ros::init(argc, argv, "proxemics_evaulation");

    // instantiating ROS node handle
    ros::NodeHandle nh;

    // instantiate the Proxemics object
    Proxemics prox(nh);

    // while (ros::ok())
    // {
    //     prox.publishProxemics();
    //     ros::spinOnce();
    // }

    // prox.publishProxemics();

    ros::spin();
    
    return 0;
}