#include <cmath>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>

const double PI = 3.14159265358979323846;

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


std::vector< std::vector<Node> > generate_roadmap(
    std::vector< std::vector<int> > road_map,
    std::vector<double> sample_x,
    std::vector<double> sample_y)
{
    std::vector< std::vector<Node> > result;
    double inf = std::numeric_limits<double>::max();

    for (auto& v : road_map)
    {
        std::vector<Node> temp = {};
        for (int idx : v)
        {
            temp.push_back(Node(sample_x[idx], 
                sample_y[idx], inf, idx, -1));
        }
        result.push_back(temp);
    }
    return result;
}


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
    Node start_node = Node(start[0], start[1], 0.0, 8, -1);
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

double gaussian(const std::vector<double>& x,
                const std::vector<double>& mu,
                const std::vector<double>& sigma)
{
    // const double a = 1.0 / (2 * PI * mu[0] * mu[1]);
    const double a = 1.0;
    return a * std::exp(-(((x[0] - mu[0])*(x[0] - mu[0])) / (2.0 * sigma[0] * sigma[0]) +
                          ((x[1] - mu[1])*(x[1] - mu[1])) / (2.0 * sigma[1] * sigma[1])));
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

    // std::cout << "Path is [ ";
    // for (auto v : path) 
    //     std::cout << v << " ";
    // std::cout << "]" << std::endl;



    // std::vector< std::vector<Node> > node_road_map =  generate_roadmap(road_map, sample_x, sample_y);

    // std::cout << "Output: [" << node_road_map[1][0].idx << ", " << node_road_map[1][1].idx << "]" << std::endl;

    // std::cout << "Output: [" << std::numeric_limits<double>::max() << std::endl;

    // std::priority_queue<Node, std::vector<Node>, compare > open;
    
    // Node n1 = Node(0, 0, 0.1, -1);
    // Node n2 = Node(0, 0, 6.5, -1);
    // Node n3 = Node(0, 0, 3.5, -1);

    // open.push(n2);
    // open.push(n3);
    // open.push(n1);
    // // open.push(0.3);

    // std::cout << "Output: [" << open.top().cost << std::endl;
    // open.pop();
    // std::cout << ", " << open.top().cost << "]" << std::endl;
    // std::cout << "Output: [" << road_map[1][0] << ", " << road_map[1][1] << "]" << std::endl;


    std::vector<double> human = {3, 2};
    std::vector<double> robot = {5, 4};

    double dist = std::hypot(robot[0]-human[0], robot[1]-human[1]);
    double theta = std::atan2(robot[1]-human[1], robot[0]-human[0]);
    double heading = std::atan2(1, 0);
    double alpha = theta-heading;

    std::cout << "[dist, theta, heading, alpha]: [ " << dist << ", " << theta 
              << ", " << heading << ", " << alpha << "]" << std::endl;

    // double px = dist*std::cos(alpha);
    // double py = dist*std::sin(alpha);
    double px = 0.349; double py = 0.481;
    double rho_x = 2.0;
    double rho_y = rho_x/1.5;
    // double A = 255 * 2 * 3.1417 * rho_x * rho_y;
    // double A = 255;
    double C = 0;
    // double spp = -(px*px*rho_y*rho_y)/(2*py*py*rho_x*rho_x);

    // double spp = -(px*px/(2*rho_x*rho_x) + py*py/(2*rho_y*rho_y));

    if (std::abs(alpha) < 1.57) {
        // C = A * 1/(2 * 3.1417 * rho_x * rho_y) * std::exp(spp);
        C = gaussian({px, py}, {0.0, 0.0}, {rho_x, rho_y});
    }

    std::cout << "[px, py, C]: [ " << px << ", " << py 
              << ", " << C << "]" << std::endl;


    
}