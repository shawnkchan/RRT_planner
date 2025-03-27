#include "ros/ros.h"
#include "rrt_planner/rrt_planner.h"


namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{

  // TODO: Fill out this function to:
  // Get map and path topics from parameter server
  // Subscribe to map topic
  map_sub_ = nh_->subscribe("map", 10, &RRTPlanner::mapCallback, this);
  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe("initialpose", 10, &RRTPlanner::initPoseCallback, this);
  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe("move_base_simple/goal", 10, &RRTPlanner::goalCallback, this);
  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>("path", 10);
  // This loops until the node is running, will exit when the node is killed
}


void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  // TODO: Fill out this function to receive and process the current map
  ROS_INFO("Map received");
  map_received_ = true;

  // unpack map metadata (width, height, res, origin)
  const nav_msgs::MapMetaData & map_info = msg->info;
  std::vector<int8_t> & map_data = msg->data;
  map_grid_ = msg;
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  // TODO: Fill out this function to receive and process the initial position
  init_pose_received_ = true;
  const geometry_msgs::PoseWithCovariance & pose = msg->pose;
  geometry_msgs::Point init_position = pose.pose.position;
  init_pose_.x(init_position.x);
  init_pose_.y(init_position.y);
  ROS_INFO_STREAM("Init pose "<<init_pose_.x()<<","<<init_pose_.y());
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  // TODO: Fill out this function to receive and process the goal position
  goal_received_ = true;
  const geometry_msgs::Pose & pose = msg->pose;
  geometry_msgs::Point goal_position = pose.position;
  goal_.x(goal_position.x);
  goal_.y(goal_position.y);
  ROS_INFO_STREAM("Goal "<<goal_.x()<<","<<goal_.y());
}

void RRTPlanner::drawGoalInitPose()
{
  // TODO: Fill out this function to draw current and goal position on map
  drawCircle(goal_, 5, cv::Scalar(0, 0, 255));
  drawCircle(init_pose_, 5, cv::Scalar(0, 0, 255));
}

void RRTPlanner::plan()
{
  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  ROS_INFO("Starting Planning");
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
    	if (map_received_ && goal_received_ && init_pose_received_) {
          	ROS_INFO("Map, goal, and init_pose received. Building map image");
			buildMapImage();
  		}
    	ros::spinOnce();
        loop_rate.sleep();
    }
}

void RRTPlanner::publishPath()
{
  // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
  return map_->at<uchar>(p.x(), p.y()) == 0;
}

void RRTPlanner::buildMapImage()
{
  // TODO: Fill out this function to create a cv::Mat object from nav_msgs::OccupancyGrid message
  if (map_received_) {
    int height = map_grid_->info.height;
  	int width = map_grid_->info.width;
  	map_ = std::unique_ptr<cv::Mat>(new cv::Mat(height, width, CV_8UC1));
	for (int x = 0; x < height ; x++) {
		for (int y = 0; y < width; y++) {
  			int index = RRTPlanner::toIndex(x, y);
  			// mirror along the horizontal axis since occupancyGrid starts from top to bottom, left to right and mat is completely opposite
  			map_->at<uchar>(height - x - 1, y) = map_grid_->data[index];
		}
	}
    drawGoalInitPose();
  	ROS_INFO("Displaying map");
  	displayMapImage(1);
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}

}  // namespace rrt_planner
