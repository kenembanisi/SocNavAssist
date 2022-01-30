#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Twist2D.h"
#include "sarvo_msgs/Person.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "sarvo_local_planner/utilities.h"
#include <stack>
#include <unordered_map>
#include <opencv2/flann.hpp>
#include <opencv2/opencv.hpp>
#include <random>


namespace sarvo_local_planner {

using namespace sarvo_msgs;

struct Node {
    double x;
    double y;
    double cost;
    int idx;
    int parent_idx;

    Node() {};
    Node(double x, double y, double cost, int idx, int p_idx) : 
        x(x), y(y), cost(cost), idx(idx), parent_idx(p_idx) {};
};

struct compare {
    bool operator()(const Node& a, const Node& b) { return a.cost > b.cost; }
};

/* Derived from CtfChan: https://github.com/CtfChan/LearnRoboticsCpp/blob/master/include/path_planning/prm.hpp */
class KDTree {

    private:
        cv::flann::Index tree_;
        cv::Mat_<float> features_;

    public:
        KDTree() {};

        KDTree(std::vector<float>& x_points, std::vector<float>& y_points)
        {
            cv::Mat_<float> features(0, 2);
            for (size_t i = 0; i < x_points.size(); ++i) {
                cv::Mat row = (cv::Mat_<float>(1,2) << x_points[i], y_points[i]);
                features.push_back(row);
            }

            tree_.build(features,
                        cv::flann::KDTreeIndexParams(1),
                        cvflann::FLANN_DIST_EUCLIDEAN);

            features_ = features;
        }

        std::pair<std::vector<int>, std::vector<float>> 
            knnSearch(float x, float y, int knn=1)
        {
            cv::Mat query = (cv::Mat_<float>(1, 2) << x, y);
            cv::Mat indices, dists; 

            tree_.knnSearch(query, indices, dists, knn, cv::flann::SearchParams(32));

            std::vector<int> indices_vec;
            std::vector<float> dists_vec;

            for (int i = 0; i < knn; ++i) {
                indices_vec.push_back( indices.at<int>(0,i) );
                dists_vec.push_back( dists.at<float>(0,i) );
            }

            return {indices_vec, dists_vec};
        }

};


class PathPlanner {


    public:

        PathPlanner(costmap_2d::Costmap2D* costmap, 
            const Pose2D& goal_pose,
            const Pose2D& start_pose,
            const double robot_fov,
            const double config_space_step_size,
            const double connecting_distance_threshold,
            std::vector<double> sample_x,
            std::vector<double> sample_y);

        ~PathPlanner();

        std::vector<std::vector<Node>> generateRoadMap(
            std::vector< std::vector<int> > adjacency_list);

        std::vector<std::vector<int>> generateRoadMap(
            std::vector<float>& samples_x, std::vector<float>& samples_y);

        std::pair<std::vector<float>, std::vector<float>> generateSamples();

        void updateRoadMapwithGoal();

        std::stack<Pose2D> computePathToGoal(const Pose2D& robot_pose);

        bool isWayPointVisible(const Pose2D& robot_pose,
            const Pose2D& waypoint);

        bool isWayPointReached(const Pose2D& robot_pose,
            const Pose2D& waypoint);

        bool validEdge(const float& current_x, const float& current_y,
            const float& next_x, const float& next_y);

        bool checkCollision(const Pose2D& pose);

        bool checkCollision(const float& x, const float& y);

        Pose2D projectPosition(const Pose2D& pos,
            const std::vector<double>& dir,
            const double mag);

        std::pair<float, float> projectPosition(const float& x,
            const float &y, 
            const std::vector<double>& dir, 
            const double mag);

        std::stack<Pose2D> dijkstraPlanner(const Pose2D& next_wp,
            const int& next_wp_idx);

        std::stack<Pose2D> AStarPlanner(const Pose2D& next_wp,
            const int& next_wp_idx);

        bool inBetween(double theta_v,
            double theta_left, double theta_right);

        bool inBetween2Pi(double theta_v,
            double theta_left, double theta_right);

        double nodeDist(const Node& point1, const Node& point2);

        void worldToMap();


    private:

        costmap_2d::Costmap2D* static_costmap_;
        Pose2D goal_, start_;
        std::vector<double> sample_x_, sample_y_;
        std::vector<float> new_samples_x_, new_samples_y_;
        double robot_fov_, step_size_, connecting_dist_threshold_;
        std::vector<std::vector<Node>> prm_roadmap_;
        std::vector<double> van_der_corput_seq_ = 
            {1, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 
            0.875, 0.0625, 0.5625};
        double waypoint_threshold_ = 0.5;
        int num_sample_points_ = 35;
        int max_num_neighbors_ = 5;
        KDTree* sample_kdtree_;

        

};






}