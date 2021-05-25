#include "ros/ros.h"
#include "haptic_interface/haptic_interface.h"

/**
 * main function
 */
int main(int argc, char **argv){

    // instantiate the node
    ros::init(argc, argv, "falcon_teleop_control");

    // ros node initialization
    ros::NodeHandle n;

    // instantiate haptic control object
    FalconNovintControl base_control = FalconNovintControl(n);

    // run control
    base_control.commandVelocity();

    ros::spin(); // keep loop alive until its shutdown

}