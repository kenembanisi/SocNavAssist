#include <sarvo_local_planner/path_planner.h>

namespace sarvo_local_planner {


PathPlanner::PathPlanner(costmap_2d::Costmap2D* costmap, 
    const Pose2D& goal_pose,
    const Pose2D& start_pose,
    const double robot_fov,
    const double config_space_step_size,
    const double connecting_distance_threshold,
    std::vector<double> sample_x,
    std::vector<double> sample_y) :
    static_costmap_(costmap), goal_(goal_pose), start_(start_pose),
    robot_fov_(robot_fov), step_size_(config_space_step_size),
    connecting_dist_threshold_(connecting_distance_threshold),
    sample_x_(sample_x), sample_y_(sample_y)
{
    // set adjacency list
    std::vector< std::vector<int> > adjacency_list = {
        {1,5}, {0,2,3,4}, {1,3,4}, {2,4,8}, {1,5,6,7}, {0,4,6,7}, 
        {5,7,11}, {4,6,8,10}, {3,7,9}, {8,12}, {7,11,12}, {6,10,12}, {}
    };

    // convert samples to map_frame [Temporary]
    worldToMap();

    // // generate prm
    // prm_roadmap_ = generateRoadMap(adjacency_list);

    auto [samples_x, samples_y] = generateSamples();

    new_samples_x_ = samples_x;
    new_samples_y_ = samples_y;


    auto road_map = generateRoadMap(samples_x, samples_y);

    printVectorOfVector(road_map);

    prm_roadmap_ = generateRoadMap(road_map);

    // printVectorOfNode(prm_roadmap_);

    ROS_INFO("/////////////////////////////");

    updateRoadMapwithGoal();

    // ROS_INFO("Size: [%d]; %d : [%f, %f]", (int)prm_roadmap_.size(), 
    //     prm_roadmap_[2][0].idx, prm_roadmap_[2][0].x, prm_roadmap_[2][0].y);
    // for (size_t i = 0; i < samples_x.size(); ++i)
    //     std::cout << "[ " << samples_x[i] << ", " << samples_y[i] << " ] , \n";

    // printVectorOfNode(prm_roadmap_);
}

PathPlanner::~PathPlanner()
{
    ROS_ERROR("Destroying the path planner object");
    delete sample_kdtree_;
}

std::vector<std::vector<Node>> PathPlanner::generateRoadMap(
    std::vector< std::vector<int> > adjacency_list)
{
    std::vector< std::vector<Node> > result;

    for (auto& v : adjacency_list)
    {
        std::vector<Node> temp = {};
        for (int idx : v)
        {
            temp.push_back(Node(new_samples_x_[idx], 
                new_samples_y_[idx], INF, idx, -1));
        }
        result.push_back(temp);
    }
    return result;
}


std::vector<std::vector<int>> PathPlanner::generateRoadMap(
    std::vector<float>& samples_x, std::vector<float>& samples_y)
{    
    // for each sampled point, find the n nearest neighbors
    size_t n_samples = samples_x.size();
    std::vector<std::vector<int>> road_map;
    road_map.reserve(n_samples);

    // generate kd-tree from samples
    // KDTree sample_kdtree(samples_x, samples_y);
    sample_kdtree_ = new KDTree(samples_x, samples_y);

    for (int i = 0; i < n_samples; ++i) {
        float x = samples_x[i];
        float y = samples_y[i];

        // sort all other samples by distance to current sample
        auto [indices, dists] = sample_kdtree_->knnSearch(x, y, n_samples);

        // find the n closest neighbors with valid edges
        std::vector<int> edge_id;
        edge_id.reserve(max_num_neighbors_);
        for (size_t j = 0; j < indices.size(); ++j) {
            float nx = samples_x[indices[j]];
            float ny = samples_y[indices[j]];

            // check if valid edge exists
            if (validEdge(x, y, nx, ny) && i != indices[j])
                edge_id.push_back(indices[j]);

            // check if max number of neighbors is reached
            if (edge_id.size() >= max_num_neighbors_)
                break;
        }

        road_map.push_back(edge_id);
    }

    // // compute neighbors for the goal node and add to road_map
    //     // sort all other samples by distance to current sample
    // auto [indices, dists] = sample_kdtree_->knnSearch((float)goal_.x, (float)goal_.y, n_samples);

    // // find the n closest neighbors with valid edges
    // std::vector<int> edge_id;
    // edge_id.reserve(max_num_neighbors_);
    // for (size_t j = 0; j < indices.size(); ++j) {
    //     float nx = samples_x[indices[j]];
    //     float ny = samples_y[indices[j]];

    //     // check if valid edge exists
    //     if (validEdge((float)goal_.x, (float)goal_.y, nx, ny))
    //         edge_id.push_back(indices[j]);

    //     // check if max number of neighbors is reached
    //     if (edge_id.size() >= max_num_neighbors_)
    //         break;
    // }
    
    return road_map;
}


std::pair<std::vector<float>, std::vector<float>> 
    PathPlanner::generateSamples()
{

    // create vectors for samples
    std::vector<float> samples_x;
    std::vector<float> samples_y;
    samples_x.reserve(num_sample_points_);
    samples_y.reserve(num_sample_points_);

    // get the boundaries of the world
    // double min_x = static_costmap_->getOriginX();
    // double min_y = static_costmap_->getOriginY();
    // double max_x = static_costmap_->getSizeInMetersX() + min_x;
    // double max_y = static_costmap_->getSizeInMetersY() + min_y;
    float min_x = -0.15f;
    float min_y = -4.20f;
    float max_x = 14.60f;
    float max_y = 5.86f; // these are values for the hall_map. Will be different for other maps

    std::vector<float> samples_x_tmp, samples_y_tmp;

    for (int i = 0; i < int(std::abs(max_x - min_x)/2); ++i)
        samples_x_tmp.push_back(min_x + 1 + 2*i);
    for (int i = 0; i < int(std::abs(max_y - min_y)/2); ++i)
        samples_y_tmp.push_back(min_y + 1 + 2*i);

    for (auto tx : samples_x_tmp) {
        for (auto ty : samples_y_tmp) {
            // check that sample is collision-free
            if (!checkCollision(tx, ty)) {
                samples_x.push_back(tx);
                samples_y.push_back(ty);
            }
        }
    }


    return {samples_x, samples_y};

}


void PathPlanner::updateRoadMapwithGoal()
{
    float x = float(goal_.x);
    float y = float(goal_.y);

    int goal_idx = new_samples_x_.size();

    // sort all other samples by distance to current sample
    auto [indices, dists] = sample_kdtree_->knnSearch(x, y, max_num_neighbors_);

    // find the n closest neighbors with valid edges
    int n_goal_neighbors = 0;
    for (size_t j = 0; j < indices.size(); ++j) {
        float nx = new_samples_x_[indices[j]];
        float ny = new_samples_y_[indices[j]];

        // check if valid edge exists
        if (validEdge(x, y, nx, ny)) {
            prm_roadmap_[indices[j]].push_back(Node(goal_.x, 
                    goal_.y, 0.0, goal_idx, -1));
            n_goal_neighbors++;
        }

        // check if max number of neighbors is reached
        if (n_goal_neighbors >= max_num_neighbors_)
            break;
    }

    // prm_roadmap_.push_back({}); // no need to add the goal now

}


std::stack<Pose2D> PathPlanner::computePathToGoal(
    const Pose2D& robot_pose)
{
    ROS_INFO("[computePathToGoal]: Computing new path...");
    double min_dist = INF;
    Pose2D next_waypoint;
    bool found_waypoint = false;
    double dist_to_waypoint, waypoint_to_goal;
    int node_idx;

    /* TODO: Use a KD-Tree here to reduce the complexity from O(n) to O(log n)? 
        A query function can be used to return all samples within a distance r */

    auto [indices, dists] = sample_kdtree_->knnSearch(robot_pose.x, robot_pose.y, 8); 
        // 8 is set as an arbitrary number to search around

    // printVector(indices);

    for (size_t idx = 0; idx < indices.size(); ++idx)
    {
        Pose2D node_pose;
        node_pose.x = new_samples_x_[indices[idx]];
        node_pose.y = new_samples_y_[indices[idx]];

        dist_to_waypoint = std::sqrt(dists[indices[idx]]);
        waypoint_to_goal = abs(goal_, node_pose);

        // check distance to way_point is under threshold
        if (dist_to_waypoint > connecting_dist_threshold_) break;

        // std::cout << "Index: " << indices[idx] << ", has cost: " << dist_to_waypoint + waypoint_to_goal << "\n";

        // check if it is visible, i.e. a valid edge exists
        if (isWayPointVisible(robot_pose, node_pose) 
            && (dist_to_waypoint + waypoint_to_goal) < min_dist)
        {
            next_waypoint = node_pose;
            min_dist = dist_to_waypoint + waypoint_to_goal;
            node_idx = indices[idx];
            found_waypoint = true;
        }
    }

    // for (int idx = 0; idx < sample_y_.size(); ++idx)
    // {
    //     Pose2D node_pose;
    //     node_pose.x = sample_x_[idx];
    //     node_pose.y = sample_y_[idx];

    //     dist_to_waypoint = abs(robot_pose, node_pose);
    //     waypoint_to_goal = abs(goal_, node_pose);

    //     // ROS_INFO("[computePathToGoal]: Waypoint [%d] checked...", idx);

    //     // check distance to way_point is under threshold
    //     if (dist_to_waypoint > connecting_dist_threshold_) continue;
        
    //     /* TODO: Include orientation to the heuristic of waypoints?  */
    //     if (isWayPointVisible(robot_pose, node_pose) 
    //         && (dist_to_waypoint + waypoint_to_goal) < min_dist)
    //     {
    //         next_waypoint = node_pose;
    //         min_dist = dist_to_waypoint + waypoint_to_goal;
    //         node_idx = idx;
    //         found_waypoint = true;
    //     }
    // }

    if (found_waypoint) {
        // append roadmap with new start vertex and edge
        // prm_roadmap_.push_back( {Node(next_waypoint.x, 
        //             next_waypoint.y, INF, node_idx, -1)} );

        std::cout << "[computePathToGoal]: Next Waypoint is: " << node_idx << ": [" << next_waypoint.x << ", " 
        << next_waypoint.y << "]" << std::endl;

        return dijkstraPlanner(next_waypoint, node_idx);
    }
    else {
        // instantiate return path stack
        std::stack<Pose2D> path_stack;
        Pose2D waypoint;
        waypoint.x = goal_.x;
        waypoint.y = goal_.y;
        path_stack.push(waypoint);
        ROS_INFO("***************************************************");
        ROS_INFO("[computePathToGoal]: No visible waypoint found. No path generated! Heading to Goal!");
        ROS_INFO("***************************************************");
        return path_stack;
    }

    
}


std::stack<Pose2D> PathPlanner::dijkstraPlanner(const Pose2D& next_wp,
    const int& next_wp_idx)
{
    bool path_found = false;

    // instantiate open and closed sets
    std::priority_queue<Node, std::vector<Node>, compare > open_set;
    std::unordered_map<int, Node> closed_set;

    // add the start node into the open set
    // Node start_node = Node(next_wp.x, next_wp.y, 0.0, (int)sample_x_.size(), -1);
    Node start_node = Node(next_wp.x, next_wp.y, 0.0, next_wp_idx, -1);
    // Node goal_node = Node(goal_.x, goal_.y, 0.0, (int)sample_x_.size()-1, -1);
    Node goal_node = Node(goal_.x, goal_.y, 0.0, (int)new_samples_x_.size(), -1);

    std::cout << "[dijkstraPlanner]: Goal node is: " << goal_node.idx << ": [" << goal_node.x << ", " 
        << goal_node.y << "]" << std::endl;
    std::cout << "[dijkstraPlanner]: Current waypoint node is: " << start_node.idx << ": [" << start_node.x << ", " 
        << start_node.y << "] and edge cost: [" << start_node.cost << "]" << std::endl;

    open_set.push(start_node);

    // main algorithm loop:
    while (!open_set.empty())
    {
        // get min node in the open set
        Node current_node = open_set.top();
        // std::cout << "[dijkstraPlanner]: Checking 'Current node': " << current_node.idx << std::endl;
        open_set.pop();
        
        // check if current_node is goal_node
        if (current_node.idx == goal_node.idx){
            goal_node.parent_idx = current_node.parent_idx;
            goal_node.cost = current_node.cost;
            path_found = true;
            std::cout << "[dijkstraPlanner]: Path to goal FOUND!" << std::endl;
            break;
        }

        // traverse from current node
        for (auto next_node : prm_roadmap_[current_node.idx])
        {
            // skip this next node if its in closed set
            if (closed_set.find(next_node.idx) != closed_set.end()) continue;

            // calculate the edge cost
            double edge_cost = nodeDist(current_node, next_node);
            
            std::cout << "[dijkstraPlanner]: 'Currnode': " << current_node.idx << ", cost: "
                << current_node.cost  << ",  'Nxtnode' : " << next_node.idx << ", cost: "
                << next_node.cost << " , push cond : "
                << (next_node.cost > (current_node.cost + edge_cost)) << std::endl;

            // update the cost of next node and place in priority queue 
            // if its cost is higher than current path
            if (next_node.cost > (current_node.cost + edge_cost)){
                next_node.cost = current_node.cost + edge_cost;
                next_node.parent_idx = current_node.idx;
                open_set.push(next_node);

                // std::cout << "[dijkstraPlanner]: Added 'Next node' : " << next_node.idx <<
                //     " to open_set of size [" << open_set.size() << "]" << std::endl;
            }            
        }

        // add current_node to closed set
        closed_set.insert( {current_node.idx, current_node} );
    }

    // instantiate return path stack
    std::stack<Pose2D> path_stack;
    Pose2D waypoint;
    waypoint.x = goal_node.x;
    waypoint.y = goal_node.y;
    path_stack.push(waypoint);

    if (path_found)
    {
        int parent_idx = goal_node.parent_idx;
        std::cout << "***************************************************" << std::endl;
        std::cout << "[dijkstraPlanner]: Node on the current path are: " << goal_node.idx << " ";
        while (parent_idx != -1)
        {
            auto node = closed_set[parent_idx];
            waypoint.x = node.x;
            waypoint.y = node.y;
            path_stack.push(waypoint);

            std::cout << parent_idx << " ";

            parent_idx = node.parent_idx;
        }
        std::cout << std::endl;
        std::cout << "***************************************************" << std::endl;
        // return path_stack;
    }
    else {
        std::cout << "***************************************************" << std::endl;
        std::cout << "[dijkstraPlanner]: Path to goal NOT found! Heading to Goal!" << std::endl;
        std::cout << "***************************************************" << std::endl;
    }
    return path_stack;
}


double PathPlanner::nodeDist(const Node& point1, const Node& point2)
{
    return std::hypot( point1.x - point2.x, point1.y - point2.y );
}


bool PathPlanner::isWayPointVisible(const Pose2D& robot_pose,
    const Pose2D& waypoint)
{   
    // ROS_INFO("[isWayPointVisible]: Checking if waypoint is visible...");
    // double fov_angle = PI/3;
    // std::vector<double> fov_range = {robot_pose.theta+robot_fov_, robot_pose.theta-robot_fov_};
    double alpha = atan(waypoint, robot_pose); // atan converts from map_frame to world frame
                                                      // and wraps to 2PI
    
    if (alpha < 0) alpha += 2*PI; // wrap 0->2*PI

    double theta_left = wrapTo2Pi( mapToGazeboAngle2Pi(robot_pose.theta) + robot_fov_ );
    double theta_right = wrapTo2Pi( mapToGazeboAngle2Pi(robot_pose.theta) - robot_fov_ );

    // ROS_INFO( "[isWayPointVisible]: [alpha, theta_l, theta_r]: [%0.3f, %0.3f, %0.3f]", 
        // alpha, theta_left, theta_right);

    /* TODO: Accomodate fov greater than 1.57 */
    if (!inBetween2Pi(alpha, theta_left, theta_right)) 
    {
        // ROS_INFO("[isWayPointVisible]: Check complete: Waypoint is NOT visible (NOT within FOV)...");
        return false;
    }

    // std::vector<double> van_der_corput_seq = {1, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, 0.0625, 0.5625};
    // double config_space_step_size_ = 0.6;

    double mag = abs(robot_pose, waypoint);
    std::vector<double> vec_dir = { (waypoint.x-robot_pose.x)/mag, (waypoint.y-robot_pose.y)/mag };
    int n_steps = mag / step_size_;
    // for (int i = 0; i < n_steps && i < van_der_corput_seq.size(); ++i){
    for (int i = 0; i < std::min(n_steps, (int)van_der_corput_seq_.size()); ++i){
        Pose2D pose = projectPosition(robot_pose, vec_dir, mag * van_der_corput_seq_[i]);
        if (checkCollision(pose)) {
            // ROS_INFO("[isWayPointVisible]: Check complete: Waypoint is NOT visible (Collision)...");
            return false;
        }
    }

    // ROS_INFO("[isWayPointVisible]: Waypoint is visible...");
    return true;
}


bool PathPlanner::isWayPointReached(const Pose2D& robot_pose,
    const Pose2D& waypoint)
{   
    double dist = abs(robot_pose, waypoint);
    if (dist < waypoint_threshold_) return true;
    else return false;
}


Pose2D PathPlanner::projectPosition(const Pose2D& pose,
    const std::vector<double>& dir, const double mag)
{
    Pose2D next_pose;
    next_pose.x = pose.x + dir[0] * mag;
    next_pose.y = pose.y + dir[1] * mag;
    return next_pose;
}


std::pair<float, float> PathPlanner::projectPosition(const float& x,
    const float &y, const std::vector<double>& dir, const double mag)
{
    float next_x = x + dir[0] * mag;
    float next_y = y + dir[1] * mag;
    return {next_x, next_y};
}


bool PathPlanner::validEdge(const float& current_x, const float& current_y,
    const float& next_x, const float& next_y)
{
    double mag = std::hypot(current_x-next_x, current_y-next_y);

    std::vector<double> vec_dir = { (next_x-current_x)/mag, (next_y-current_y)/mag };

    int n_steps = mag / step_size_;
    
    for (int i = 0; i < std::min(n_steps, (int)van_der_corput_seq_.size()); ++i) {
        auto [nx, ny] = projectPosition(current_x, current_y, vec_dir, mag * van_der_corput_seq_[i]);

        if (checkCollision(nx, ny)) {
            // ROS_INFO("[isWayPointVisible]: Check complete: Waypoint is NOT visible (Collision)...");
            return false;
        }
    }
    return true;
}


bool PathPlanner::checkCollision(const Pose2D& pose)
{
    unsigned int mx, my;
    static_costmap_->worldToMap(pose.x, pose.y, mx, my);
    unsigned char pose_cost = static_costmap_->getCost(mx, my);

    if (pose_cost > 200) return true;

    return false;
}


bool PathPlanner::checkCollision(const float& x, const float& y)
{
    unsigned int mx, my;
    static_costmap_->worldToMap(x, y, mx, my);
    unsigned char pose_cost = static_costmap_->getCost(mx, my);

    if (pose_cost > 10) return true;

    return false;
}


bool PathPlanner::inBetween(double theta_v,
    double theta_left, double theta_right)
{
    // ROS_INFO("Getting here, inside inBetween!... [%0.2f, %0.2f, %0.2f]",
    //     theta_v, theta_left, theta_right);

    if (std::abs(theta_right - theta_left) <= PI){ 
        if (theta_right <= theta_v && theta_v <= theta_left) return true;
        else return false;
    }
    else {
        if (theta_left < 0 && theta_right > 0){
            theta_left += 2*PI;
            if (theta_v < 0) theta_v += 2*PI;
            if (theta_v >= theta_right && theta_v <= theta_left) return true;
            else return false;
        }
        if (theta_left > 0 && theta_right < 0){
            theta_right += 2*PI;
            if (theta_v < 0) theta_v += 2*PI;
            if (theta_v <= theta_right && theta_v >= theta_left > 0) return true;
            else return false;
        }
    }
}


bool PathPlanner::inBetween2Pi(double theta_v,
    double theta_left, double theta_right)
{   
    if (theta_right < 0) theta_right += 2*PI;
    if (std::abs(theta_right - theta_left) <= PI){ 
        if (theta_v >= theta_right && theta_v <= theta_left) return true;
        else return false;
    }
    else {
        if ((theta_v <= theta_right && theta_v <= theta_left) ||
            (theta_v >= theta_right && theta_v >= theta_left)) return true;
            else return false;
    }   
}


void PathPlanner::worldToMap()
{
    std::vector<double> sample_x_t, sample_y_t;
    for (std::size_t i = 0; i < sample_x_.size(); ++i){
        sample_x_t.push_back(sample_y_[i] + 6.80);
        sample_y_t.push_back(-sample_x_[i] - 5.52);
    }
    sample_x_ = sample_x_t;
    sample_y_ = sample_y_t;
}


}