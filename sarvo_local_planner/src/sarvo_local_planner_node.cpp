#include <sarvo_local_planner/sarvo_local_planner.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "sarvo_planner_node");

	tf::TransformListener tf;
    tf2_ros::Buffer buffer(ros::Duration(10));
    // tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", buffer);

	// namespace::ClassName object_name
	sarvo_local_planner::SARVOLocalPlanner sarvo_planner(buffer, costmap_ros);

	ros::spin();

	return 0;

}