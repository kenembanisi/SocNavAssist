#include <sarvo_local_planner/trajectory_critic.h>

// Credits: https://github.com/locusrobotics/robot_navigation/blob/noetic/dwb_critics/src/base_obstacle.cpp
// Credits: https://github.com/makokal/socially_normative_navigation/blob/master/behavior_functions/include/behavior_functions/social_compliance_cost.h

namespace sarvo_local_planner {


TrajectoryCritic::TrajectoryCritic(
    costmap_2d::Costmap2D* costmap,
    const std::vector<double> weights,
    const double horizon,
    const double clearance_thr,
    const double sim_granularity,
    const bool sum_obs_scores_,
    const bool sum_social_dis_scores_,
    const bool decay_social_dis_scores_) :
    static_costmap_(costmap),
    weights_(weights), 
    horizon_(horizon),
    clearance_threshold_(clearance_thr),
    sim_granularity_(sim_granularity),
    sum_obstacle_scores_(sum_obs_scores_),
    sum_social_disturbance_scores_(sum_social_dis_scores_),
    decay_social_disturbance_scores_(decay_social_dis_scores_)
{
    // initialize feature counts
    feature_counts_ = std::vector<double>(weights_.size(), 0.0);
}


void TrajectoryCritic::computeCandidateScore(Candidate& candidate, 
    const std::string& feature_name)
{
    // Compute the base obstacle cost using the cost map
    double obstacle_score = baseObstacleScore(candidate);

    // Update the candidate's raw scores
    std::vector<double> raw_scores(weights_.size(), 0.0);
    candidate.score.raw_scores = raw_scores;
    candidate.score.raw_scores[0] = obstacle_score;

}


double TrajectoryCritic::socialDisturbanceScore(const Candidate& candidate, 
    const std::vector<Person>& ped_groups)
{
    std::vector<Person> pedestrians_proj;
    double score = 0.0;
    double dt = 0.0;
    int num_steps = horizon_ / sim_granularity_;
    for (int i = 0; i < num_steps; i++)
    {
        pedestrians_proj.clear();
        for (const auto& p : ped_groups) {
            pedestrians_proj.push_back( constantVelocityProjection(p, dt) );
        }

        double sc = socialDisturbanceScore(candidate.traj.poses[i], pedestrians_proj);

        // 
        if (decay_social_disturbance_scores_)
            score += scoreDecay(sc, horizon_, dt);
            // score += scoreDecay(sc, horizon_, i * sim_granularity_);
        else
            score += sc;
        
        // ROS_INFO("dt is: [%f]", dt);

        dt += sim_granularity_;
    }

    // return score / (double)num_steps;
    return score;
}



double TrajectoryCritic::socialDisturbanceScore(const Pose2D& robot_pose,
    const std::vector<Person>& pedestrians)
{
    double score = 0.0;
    for (const auto& p : pedestrians)
    {
        double dist = abs(robot_pose, p.pose);
        double sigma = 0.1;
        if (dist < clearance_threshold_) {
            double ped_score = (clearance_threshold_ - dist);     // include a decay here to account for position uncertainty?
            // double ped_score += gaussianPDF(dist, 0.0, sigma);
            score = std::max(score, ped_score);
        }
    }
    // return score / (double)pedestrians.size();
    return score;
}
        


double TrajectoryCritic::baseObstacleScore(const Candidate& candidate)
{
    double score = 0.0;
    const costmap_2d::Costmap2D& costmap = *static_costmap_;
    for (auto& pose : candidate.traj.poses)
    {
        double pose_score = scorePose(costmap, pose);
        // check if any pose collides
        // if (pose_score > 250) return 255;
        if (sum_obstacle_scores_)
            score += pose_score;
        else
            score = std::max(score, pose_score);
    }
    return score;
}


double TrajectoryCritic::scorePose(const costmap_2d::Costmap2D& costmap,
    const Pose2D& pose)
{
    unsigned int mx, my;
    costmap.worldToMap(pose.x, pose.y, mx, my);

    unsigned char pose_cost = costmap.getCost(mx, my);

    return pose_cost;
    
}


Person TrajectoryCritic::constantVelocityProjection(const Person& person,
    const double dt)
{
    Person ped;
    ped.pose.x = person.pose.x + person.velocity.x * dt;
    ped.pose.y = person.pose.y + person.velocity.y * dt;
    return ped;
}


void TrajectoryCritic::computeTotalScore(Candidate& candidate)
{
    // std::cout << "Size of raw_scores: " << candidate.score.raw_scores.size() << std::endl;
    candidate.score.total = vdot(candidate.score.raw_scores, weights_);
}


double TrajectoryCritic::scoreDecay(const double score, 
    const int horizon, const double dt)
{
    return (1 - (dt / (2.0*horizon)));
}



void TrajectoryCritic::calculateFeatureCounts(Candidate optimal_candidate)
{
    for (size_t i = 0; i < optimal_candidate.score.raw_scores.size(); ++i)
        feature_counts_[i] += optimal_candidate.score.raw_scores[i];
    iteration_count_++;
}

}