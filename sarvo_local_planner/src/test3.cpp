#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>

const double PI = 3.14159;
const double INF = std::numeric_limits<double>::max();

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


double dist(double ux, double uy, double vx, double vy){
    return std::hypot( std::abs(ux-vx), std::abs(uy-vy) );
}


bool inBetween(double theta_v,
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


/* generate_roadmap function */
std::vector< std::vector<Node> > generate_roadmap(
    std::vector< std::vector<int> > road_map,
    std::vector<double> sample_x,
    std::vector<double> sample_y)
{
    std::vector< std::vector<Node> > result;
    // double inf = std::numeric_limits<double>::max();

    for (auto& v : road_map)
    {
        std::vector<Node> temp = {};
        for (int idx : v)
        {
            temp.push_back(Node(sample_x[idx], 
                sample_y[idx], INF, idx, -1));
        }
        result.push_back(temp);
    }
    return result;
}


/* dijkstra algorithm function */
std::vector<int> dijkstra(std::vector<double> start,
    std::vector<double> goal,
    std::vector< std::vector<Node> > road_map,
    std::vector<double> sample_x,
    std::vector<double> sample_y)
{
    bool path_found = false;

    // instantiate open and closed sets
    std::priority_queue<Node, std::vector<Node>, compare > open_set;
    std::unordered_map<int, Node> closed_set;

    // add the start node into the open set
    Node start_node = Node(start[0], start[1], 0.0, 10, -1);
    Node goal_node = Node(goal[0], goal[1], 0.0, 9, -1);
    open_set.push(start_node);

    // main algorithm loop:
    while (!open_set.empty())
    {
        // get min node in the open set
        Node current_node = open_set.top();
        open_set.pop();
        
        // check if current_node is goal_node
        if (current_node.idx == goal_node.idx){
            goal_node.parent_idx = current_node.parent_idx;
            goal_node.cost = current_node.cost;
            path_found = true;
            break;
        }

        // traverse from current node
        for (auto& next_node : road_map[current_node.idx])
        {
            // skip this next node if its in closed set
            if (closed_set.find(next_node.idx) != closed_set.end()) continue;

            // calculate the edge cost
            double edge_cost = dist(current_node.x, current_node.y,
                next_node.x, next_node.y);
            
            // update the cost of next node and place in priority queue 
            // if its cost is higher than current path
            if (next_node.cost > current_node.cost + edge_cost){
                next_node.cost = current_node.cost + edge_cost;
                next_node.parent_idx = current_node.idx;
                open_set.push(next_node);
            }            
        }

        // add current_node to closed set
        closed_set.insert( {current_node.idx, current_node} );
    }

    if (path_found)
    {
        // std::vector< std::vector<int> > path = { {goal_node.x, goal_node.y} };
        // int parent_idx = goal_node.parent_idx;
        // while (parent_idx != -1)
        // {
        //     Node node = closed_set[parent_idx];
        //     path.push_back( {node.x, node.y} );
        //     parent_idx = node.parent_idx;
        // }
        // return path;
        std::vector<int> path = {goal_node.idx};
        int parent_idx = goal_node.parent_idx;
        while (parent_idx != -1)
        {
            // Node node = closed_set[parent_idx];
            auto node = closed_set[parent_idx];
            path.push_back(parent_idx);
            parent_idx = node.parent_idx;
        }
        return path;
    }
    else return {};
}


std::vector<double> projectPosition(std::vector<double> pos,
    std::vector<double> dir,
    double mag)
{
    return { pos[0] + dir[0] * mag, pos[1] + dir[1] * mag,};
}


bool checkCollision(std::vector<double> pos)
{
    // Limits for the table 1 ---------------------------------------------------------
                    
    // table1_x_limits = [-5.77, -3.88]
    // table1_y_limits = [-4.17, -2.42]

    // if (table1_x_limits[0] < future_state_x < table1_x_limits[1]) and  \
    //     (table1_y_limits[0] < future_state_y < table1_y_limits[1]):
    //     collision_free = False

    std::vector<double> table1_x_limits = {-5.77, -3.88};
    std::vector<double> table1_y_limits = {-4.17, -2.42};

    if (table1_x_limits[0] < pos[0] && pos[0] < table1_x_limits[1] &&
        table1_y_limits[0] < pos[1] && pos[1] < table1_y_limits[1])
        return true;

    return false;
}


double wrapToPi(double angle)
{
    double x = std::fmod(angle + PI, 2*PI);
    if (x < 0) x += 2*PI;
    return x - PI;
}


/* isWayPointVisible function */
bool isWayPointVisible(std::vector<double> robot_pose,
    std::vector<double> way_point)
{
    // step 1. Is the waypoint within the field-of-view
    // step 2. Is the waypoint occluded by or within an obstacle

    // step 1:
        // 1a. theta is the robot's heading
        // 1b. fov_range = [theta-fov_angle, theta+fov_angle] where fov_angle 
        // 1c. compute alpha = atan2(robot_pose, way_point)
        // 1d. if not inbetween(alpha, fov_range) return false
    double fov_angle = PI/3;
    std::vector<double> fov_range = {robot_pose[2]+fov_angle, robot_pose[2]-fov_angle};
    double alpha = std::atan2(way_point[1]-robot_pose[1], way_point[0]-robot_pose[0]);
    if (!inBetween(alpha, wrapToPi(fov_range[0]), wrapToPi(fov_range[1]))) return false;


    // step 2:  
        // 2a. compute distance to waypoint, mag = dist(robot_pose, way_point)
        // 2b. compute vec_dir, (unit vector)
        // 2c. compute n_steps = mag / config_space_step_size
        // -- van_der_corput_seq = [1, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875]
        // 2d. for i -> 0 to n_steps
            // compute point = projectPosition(robot_pose, vec_dir, mag * van_der_corput_seq[i])
            // if checkCollision(point) return false
        // 2e. return true
    std::vector<double> van_der_corput_seq = {1, 0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, 0.0625, 0.5625};
    double config_space_step_size = 0.6;

    double mag = dist(robot_pose[0], robot_pose[1], way_point[0], way_point[1]);
    std::vector<double> vec_dir = { (way_point[0]-robot_pose[0])/mag, (way_point[1]-robot_pose[1])/mag };
    int n_steps = mag / config_space_step_size;
    // for (int i = 0; i < n_steps && i < van_der_corput_seq.size(); ++i){
    for (int i = 0; i < std::min(n_steps, (int)van_der_corput_seq.size()); ++i){
        std::vector<double> point = projectPosition(robot_pose, vec_dir, mag * van_der_corput_seq[i]);
        if (checkCollision(point)) return false;
    }

    return true;
}


/* computePathToGoal function */
std::vector<int> computePathToGoal(std::vector< std::vector<Node> > road_map,
    std::vector<double> robot_pos, 
    std::vector<double> goal,
    std::vector<double> sample_x,
    std::vector<double> sample_y)
{
    double min_dist = std::numeric_limits<double>::max();
    std::vector<double> next_waypoint;
    bool found_waypoint = false;
    double dist_to_waypoint, waypoint_to_goal, dist_threshold = 6.0;
    int node_idx;
    for (int idx = 0; idx < sample_y.size(); ++idx)
    {
        std::vector<double> node_pos = {sample_x[idx], sample_y[idx]};
        dist_to_waypoint = dist(robot_pos[0], robot_pos[1], node_pos[0], node_pos[1]);
        waypoint_to_goal = dist(node_pos[0], node_pos[1], goal[0], goal[1]);

        // check distance to way_point is under threshold
        if (dist_to_waypoint > dist_threshold) continue;

        if (isWayPointVisible(robot_pos, node_pos) 
            && (dist_to_waypoint + waypoint_to_goal) < min_dist)
        {
            next_waypoint = node_pos;
            min_dist = dist_to_waypoint + waypoint_to_goal;
            node_idx = idx;
            found_waypoint = true;
        }
    }

    if (found_waypoint) {
        // append roadmap with new start vertex and edge
        road_map.push_back( {Node(next_waypoint[0], 
                    next_waypoint[1], INF, node_idx, -1)} );
    
        return dijkstra(next_waypoint, goal, road_map, sample_x, sample_y);
    }

    return {};
}


void worldToMap(std::vector<double> sample_x_, 
    std::vector<double> sample_y_,
    std::vector<double>& sample_x_t,
    std::vector<double>& sample_y_t)
{
    for (std::size_t i = 0; i < sample_x_.size(); ++i){
        sample_x_t.push_back(sample_y_[i] + 6.80);
        sample_y_t.push_back(-sample_x_[i] - 5.52);
    }
}



int main() {

    // std::vector<double> start = {-5.52, -6.80};
    // std::vector<double> goal = {-6.48, 7.22};
    // std::vector<double> sample_x = {-2.23, -7.05, -10.97, -2.54, -6.21, -10.92, -2.19, -10.19, -5.52, -6.48};
    // std::vector<double> sample_y = {-3.58, -3.12, -3.18, 0.47, 2.70, 1.57, 5.27, 5.89, -6.80, 7.22};
    // std::vector< std::vector<int> > road_map = {
    //     {3, 1},
    //     {0, 4},
    //     {5},
    //     {4, 6},
    //     {3, 5, 9},
    //     {4, 7},
    //     {9},
    //     {9},
    //     {0, 1, 2},
    //     {}
    // };


    // std::vector< std::vector<Node> > node_road_map =  generate_roadmap(road_map, sample_x, sample_y);

    // std::vector<int> path = dijkstra(start, goal, node_road_map, sample_x, sample_y);

    // std::vector<double> robot_start = {-2.52, -6.80, 2.59};

    // std::vector<int> path = computePathToGoal(node_road_map, robot_start, goal, sample_x, sample_y);




    // std::cout << "Path is [ ";
    // for (auto v : path) 
    //     std::cout << v << " ";
    // std::cout << "]" << std::endl;

    // std::cout << wrapToPi(4.713) << std::endl;

    // std::vector<double> sample_x_t, sample_y_t;
    // worldToMap(sample_x, sample_y, sample_x_t, sample_y_t);

    // std::cout << "Samples are [ ";
    // for (std::size_t i = 0; i < sample_x.size(); ++i) {
    //     std::cout << "(" << sample_x_t[i] << ", " << sample_y_t[i] << ") ";
    // }
    // std::cout << "]" << std::endl;

    std::vector<std::vector<float>> gazebo_poses = {
        {-6.47, 6.96, 0.0},
        {-6.68, -4.22, 0.0}
    };

    for (auto& gazebo_pose : gazebo_poses) {
        float map_x = gazebo_pose[1] + 6.80;
        float map_y = -gazebo_pose[0] - 5.52;

        float map_theta = 0;
        if (gazebo_pose[2] >= -PI/2 && gazebo_pose[2] <= PI) map_theta = gazebo_pose[2] - PI/2;
        else map_theta = 1.5*PI + gazebo_pose[2];

        std::cout << "For gazebo: ";
        for (auto v : gazebo_pose) 
            std::cout << v << " ";
        std::cout << " --> ";
        std::cout << map_x << " " << map_y << " " << map_theta << std::endl;
    }



}