#include "ros/ros.h"
#include "rrt_planner/rrt_planner.h"

int main(int argv, char ** argc)
{
  ros::init(argv, argc, "rrt_planner");
  ros::NodeHandle node;
  rrt_planner::RRTPlanner planner(&node);
  ROS_INFO("Running RRTPlanner");
  planner.plan();
}
