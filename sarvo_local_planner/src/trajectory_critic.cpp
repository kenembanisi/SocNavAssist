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

std::vector<double> TrajectoryCritic::initializeScores()
{
    return std::vector<double>(weights_.size(), 0.0);
}


bool TrajectoryCritic::freeOfStaticObstacles(const Candidate& candidate)
{
    unsigned int mx, my;
    // const costmap_2d::Costmap2D& costmap = *static_costmap_;
    for (auto& pose : candidate.traj.poses)
    {
        static_costmap_->worldToMap(pose.x, pose.y, mx, my);

        unsigned char pose_cost = static_costmap_->getCost(mx, my);

        if (pose_cost == costmap_2d::LETHAL_OBSTACLE || 
            pose_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;
    }
    return true;
}

bool TrajectoryCritic::forwardHeadingFree(const Candidate& candidate)
{
    unsigned int mx, my;
    
    for (int i = candidate.traj.poses.size()-1; i > 1; --i)
    {
        Pose2D pose = candidate.traj.poses[i];
        static_costmap_->worldToMap(pose.x, pose.y, mx, my);

        unsigned char pose_cost = static_costmap_->getCost(mx, my);

        if (pose_cost == costmap_2d::LETHAL_OBSTACLE || 
            pose_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            return false;
    }
    return true;
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
        // ROS_INFO("Social score is: [%f]", sc);

        dt += sim_granularity_;
    }

    // ROS_INFO("..................................");
    // ROS_INFO("Total Social score is: [%f]", score);

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
            // score = std::max(score, ped_score);
            score += ped_score;
        }
    }
    // return score / (double)pedestrians.size();
    return score;
}


double TrajectoryCritic::socialIntrusionGaussianScore(const Candidate& candidate, 
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

        double sc = socialIntrusionGaussianScore(candidate.traj.poses[i], pedestrians_proj);

        // 
        if (decay_social_disturbance_scores_)
            score += scoreDecay(sc, horizon_, dt);
            // score += scoreDecay(sc, horizon_, i * sim_granularity_);
        else
            score += sc;
        
        // ROS_INFO("dt is: [%f]", dt);
        // ROS_INFO("Social score is: [%f]", sc);

        dt += sim_granularity_;
    }

    // ROS_INFO("..................................");
    // ROS_INFO("Total Social score is: [%f]", score);

    // return score / (double)num_steps;
    return score;
}


double TrajectoryCritic::socialIntrusionGaussianScore(const Pose2D& robot_pose,
    const std::vector<Person>& pedestrians)
{
    double score = 0.0;
    for (const auto& p : pedestrians)
    {
        double dist = abs(robot_pose, p.pose);
        double theta = std::atan2(robot_pose.y-p.pose.y, robot_pose.x-p.pose.x);
        double heading = std::atan2(p.velocity.y, p.velocity.x) / std::hypot(p.velocity.x, p.velocity.y);
        // double heading = std::atan2(1.0, 0.0);
        double alpha = theta-heading;
        double px = dist*std::cos(alpha);
        double py = dist*std::sin(alpha);

        // std::cout << "[dist, theta, heading, alpha, px, py]: [ " << dist << ", " << theta 
        //       << ", " << heading << ", " << alpha 
        //       <<  ", " << px << ", " << py << "]" << std::endl;


        std::vector<double> sigma = {1.2, 1.2/2.0};
        
        if (std::abs(alpha) < 1.57)
            score = gaussian2D({px, py}, {0.0, 0.0}, sigma);
        
    }
    // return score / (double)pedestrians.size();
    // ROS_INFO("..................................");

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


void TrajectoryCritic::normalizeRawScores(Candidate& candidate, Point2D& prev_vel,
    Point2D& goal_vel, Point2D& operator_vel)
{
    ROS_INFO("Scores: [%0.3f, %0.3f, %0.3f, %0.3f,%0.3f]",
        candidate.score.raw_scores[0],
        candidate.score.raw_scores[1],
        candidate.score.raw_scores[2],
        candidate.score.raw_scores[3],
        candidate.score.raw_scores[4]);
    
    // static cost
    candidate.score.raw_scores[0] = candidate.score.raw_scores[0] / (255 * candidate.traj.poses.size());

    // motion smoothness cost
    // candidate.score.raw_scores[1] = candidate.score.raw_scores[1] / (2 * abs(prev_vel));
    candidate.score.raw_scores[1] = candidate.score.raw_scores[1] / (2 * 2.0);

    // operator alignment cost
    // candidate.score.raw_scores[2] = candidate.score.raw_scores[2] / (2 * abs(operator_vel));
    candidate.score.raw_scores[2] = candidate.score.raw_scores[2] / (2 * 2.0);

    // goal-directed cost
    candidate.score.raw_scores[3] = candidate.score.raw_scores[3] / (2 * abs(goal_vel));
    // candidate.score.raw_scores[3] = candidate.score.raw_scores[3] / (2 * 2.0);

    // goal-directed cost
    candidate.score.raw_scores[4] = candidate.score.raw_scores[4] / (clearance_threshold_ * (horizon_ / sim_granularity_));

    ROS_INFO("Normed Scores: [%0.3f, %0.3f, %0.3f, %0.3f,%0.3f]",
        candidate.score.raw_scores[0],
        candidate.score.raw_scores[1],
        candidate.score.raw_scores[2],
        candidate.score.raw_scores[3],
        candidate.score.raw_scores[4]);
    ROS_INFO("....................................................................");
}


double TrajectoryCritic::scoreDecay(const double score, 
    const int horizon, const double dt)
{
    return (1 - (dt / (2.0*horizon)))*score;
}



void TrajectoryCritic::calculateFeatureCounts(Candidate optimal_candidate)
{
    for (size_t i = 0; i < optimal_candidate.score.raw_scores.size(); ++i)
        feature_counts_[i] += optimal_candidate.score.raw_scores[i];
    iteration_count_++;

    max_social_cost = std::max(optimal_candidate.score.raw_scores[5], max_social_cost);

    // ROS_INFO("Max social disturbance value: %0.3f", max_social_cost);
}

}