#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Twist2D.h"
#include "sarvo_msgs/Person.h"
#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "sarvo_local_planner/utilities.h"
#include <stack>
#include <unordered_map>


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

        ~PathPlanner() {}

        std::vector<std::vector<Node>> generateRoadMap(
            std::vector< std::vector<int> > adjacency_list);

        std::stack<Pose2D> computePathToGoal(const Pose2D& robot_pose);

        bool isWayPointVisible(const Pose2D& robot_pose,
            const Pose2D& waypoint);

        bool isWayPointReached(const Pose2D& robot_pose,
            const Pose2D& waypoint);

        bool checkCollision(const Pose2D& pose);

        Pose2D projectPosition(const Pose2D& pos,
            const std::vector<double>& dir,
            const double mag);

        std::stack<Pose2D> dijkstraPlanner(const Pose2D& next_wp,
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
        double robot_fov_, step_size_, connecting_dist_threshold_;
        std::vector<std::vector<Node>> prm_roadmap_;
        std::vector<double> van_der_corput_seq_ = 
            {1, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 
            0.875, 0.0625, 0.5625};
        double waypoint_threshold_ = 0.5;

        

};








}