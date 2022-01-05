
#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Pose2D.h"
#include <iostream>

using namespace sarvo_msgs;

namespace sarvo_local_planner {
	
	class VelocityObstacle {
    
    public:
        VelocityObstacle() {}

        Point2D lambda_left_;
// 
        Point2D lambda_right_;

        Point2D apex_;

        double distance_;

        double eff_obs_radius_;

    };

    double abs(const Point2D& point1, const Point2D& point2)
    {
        return std::hypot( point1.x - point2.x, point1.y - point2.y );
    }

    double abs(const Pose2D& point1, const Pose2D& point2)
    {
        return std::hypot( point1.x - point2.x, point1.y - point2.y );
    }

    double atan2m(const double y, const double x)
    {
        return std::atan2(x, -y);
    }
    
    double atan(const Pose2D& point1, const Pose2D& point2)
    {
        // return std::atan2( point1.y - point2.y, point1.x - point2.x );
        return atan2m( point1.y - point2.y, point1.x - point2.x );
    }

    template <typename T>
    void print(const std::vector<T>& vec)
    {
        for (T v : vec) 
            std::cout << v << " ";
        std::cout << std::endl;
    }


} // end namespace

#endif