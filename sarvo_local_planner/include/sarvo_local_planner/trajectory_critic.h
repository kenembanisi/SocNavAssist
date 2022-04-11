#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Twist2D.h"
#include "sarvo_msgs/Person.h"
#include "sarvo_msgs/Trajectory2D.h"
#include "sarvo_msgs/Candidate.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/cost_values.h"
#include "sarvo_local_planner/utilities.h"

/*
Credits: https://github.com/makokal/socially_normative_navigation/blob/master/behavior_functions/include/behavior_functions/social_compliance_cost.h

*/

namespace sarvo_local_planner {

using namespace sarvo_msgs;

class TrajectoryCritic {


    public:

        TrajectoryCritic(costmap_2d::Costmap2D* costmap, 
            const std::vector<double> weights,
            const double horizon,
            const double clearance_thr,
            const double sim_granularity,
            const bool sum_obstacle_scores_,
            const bool sum_social_disturbance_scores_,
            const bool decay_social_disturbance_scores_);

        ~TrajectoryCritic() {}

        void computeCandidateScore(Candidate& candidate, 
            const std::string& feature_name);
        
        bool freeOfStaticObstacles(const Candidate& candidate);

        bool forwardHeadingFree(const Candidate& candidate);

        std::vector<double> initializeScores();

        void computeCandidateScore(Candidate& candidate, 
            Point2D& prev_v_optimal_);

        double baseObstacleScore(const Candidate& candidate);

        double scorePose(const costmap_2d::Costmap2D& costmap,
            const Pose2D& pose);

        double socialDisturbanceScore(const Candidate& candidate, 
            const std::vector<Person>& ped_groups);

        double socialDisturbanceScore(const Pose2D& robot_pose,
            const std::vector<Person>& pedestrians);

        double socialIntrusionGaussianScore(const Pose2D& robot_pose,
            const std::vector<Person>& pedestrians);

        double socialIntrusionGaussianScore(const Candidate& candidate, 
            const std::vector<Person>& ped_groups);

        double computeDistance();

        Person constantVelocityProjection(const Person& person, const double dt);

        double scoreDecay(const double score, const int horizon, const double dt);

        void computeTotalScore(Candidate& candidate);

        void calculateFeatureCounts(Candidate optimal_candidate);

    private:

        std::vector<double> weights_;
        double horizon_;
        double clearance_threshold_;
        double sim_granularity_;
        bool sum_obstacle_scores_;
        bool sum_social_disturbance_scores_;
        bool decay_social_disturbance_scores_;
        costmap_2d::Costmap2D* static_costmap_;
        double max_social_cost = 0.0;
        int num_weights_;
    
    public:

        std::vector<double> feature_counts_;
        int iteration_count_ = 0;
        

};








}