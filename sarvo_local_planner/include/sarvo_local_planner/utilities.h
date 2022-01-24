
#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "sarvo_msgs/Point2D.h"
#include "sarvo_msgs/Pose2D.h"
#include <iostream>
#include <fstream>

const double PI = 3.14159265358979323846;
const double ROOT_2PI = 2.50662;
const double INF = std::numeric_limits<double>::max();

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

    double abs(const Point2D& point)
    {
        return std::hypot( point.x, point.y );
    }

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

    double wrapToPi(const double angle)
    {
        double x = std::fmod(angle + PI, 2*PI);
        if (x <= 0) x += 2*PI;
        return x - PI;
    }

    double wrapTo2Pi(const double angle)
    {
        double x = std::fmod(angle, 2*PI);
        if (x < 0) x += 2*PI;
        return x;
    }

    double mapToGazeboAngle(const double angle)
    {
        if (angle > PI/2 && angle <= PI) return angle - 1.5*PI;
        else return angle + PI/2;
    }

    double mapToGazeboAngle2Pi(const double angle)
    {
        if (angle < -PI/2 && angle >= -PI) return angle + 5*PI/2;
        else return angle + PI/2;
    }

    inline double vdot(const std::vector<double> vector1,
        const std::vector<double> vector2)
    {
        // assert(vector1.size() == vector2.size());
        double value = 0.0;
        for (int i = 0; i < vector2.size(); ++i)
            value += vector1[i] * vector2[i];
        return value;
    }

    template <typename T>
    void print(const std::vector<T>& vec)
    {
        for (T v : vec) 
            std::cout << v << " ";
        std::cout << std::endl;
    }

    void writeCSV(std::string filename, std::vector<std::string> colname)
    {
        // Create an output filestream object
        std::ofstream myFile(filename, std::fstream::app);

        if (!myFile) std::cout << "Couldn't open file" << "\n";
        else std::cout << "File opened successfully" << "\n";

        // Send column names
        for (std::string& s : colname)
            myFile << s << ",";
        myFile << "\n";

        // Close the file
        myFile.close();
    }

    void writeCSV(std::string filename, std::vector<std::string> rownames,
    std::vector<double> features)
    {
        // Create an output filestream object
        std::ofstream myFile(filename, std::fstream::app);

        if (!myFile) std::cout << "Couldn't open file" << "\n";
        else std::cout << "File opened successfully" << "\n";

        // Send row names
        for (std::string& s : rownames)
            myFile << s << ",";

        for (double f : features)
            myFile << f << ",";

        myFile << "\n";

        // Close the file
        myFile.close();
    }

    double angleBetween(const Point2D& vector1, const Point2D& vector2)
    {
        double dot_product = (vector1.x*vector2.x) + (vector1.y*vector2.y);
        double vector1_mag = std::hypot(vector1.x, vector1.y);
        double vector2_mag = std::hypot(vector2.x, vector2.y);
        return std::acos( dot_product / (vector1_mag*vector2_mag) );
    }

    double magnitudeDifference(const Point2D& vector1, const Point2D& vector2)
    {
        return std::pow(abs(vector2) - abs(vector1), 2);
    }

    double gaussianPDF(const double& x, const double& mu = 0.0, const double& sigma = 1.0)
    {
        const double a = 1.0 / (sigma * ROOT_2PI);
        return a * std::exp(-((x - mu)*(x - mu)) / (2.0 * sigma * sigma));
    }

    std::vector<double> parseWeightString(const std::string& weight_str)
    {
        std::vector<double> result;
        std::string w;
        try {
            for (char c : weight_str){
                if (c == '/' || c == ','){
                    result.push_back(std::stod(w));
                    w.clear();
                }
                else w.push_back(c);
            }    
            result.push_back(std::stod(w));
            return result;
        }
        catch (std::exception& e){
            std::cerr << "[ERROR] Error occurred with parsing weight string: " << e.what() << "\n";
            // std::cout << "String error here \n";
            return result;
        }
    }

    std::vector<double> multiply(const std::vector<double>& vec, const double val)
    {
        std::vector<double> result;
        for (size_t i = 0; i < vec.size(); ++i)
            result.push_back(vec[i] * val);
        return result;
    }


} // end namespace

#endif