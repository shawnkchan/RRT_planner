#include "ros/ros.h"
#include "rrt_planner/rrt_planner.h"
#include <nav_msgs/Path.h>
#include <cmath>

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
  const geometry_msgs::PoseWithCovariance & poseWithCov = msg->pose;
  const geometry_msgs::Pose pose = poseWithCov.pose;
  poseToPoint(init_pose_, pose);
  ROS_INFO_STREAM("Init pose "<<init_pose_.x()<<","<<init_pose_.y());
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  // TODO: Fill out this function to receive and process the goal position
  goal_received_ = true;
  const geometry_msgs::Pose & pose = msg->pose;
  poseToPoint(goal_, pose);
  ROS_INFO_STREAM("Goal "<<goal_.x()<<","<<goal_.y());
}

void RRTPlanner::drawGoalInitPose()
{
  // TODO: Fill out this function to draw current and goal position on map
  drawCircle(goal_, 10, cv::Scalar(0, 0, 255));
  drawCircle(init_pose_, 10, cv::Scalar(0, 255, 0));
}

void RRTPlanner::plan()
{
  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  ROS_INFO("Starting Planning");
  ros::Rate loop_rate(10);
  bool planning_complete = false;
  while (ros::ok())
    {
    	if (map_received_ && goal_received_ && init_pose_received_) {
          	ROS_INFO("Displaying map");
			buildMapImage();
            if (!planning_complete) {
              createTree(5, 1, 1);
              planning_complete = true;
            }
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
  // check the original map_grid_ object, 0 means unoccupied, 100 means occupied
  // returns true if point is unoccupied, false otherwise
  int8_t map_grid_value = map_grid_->data[toIndex(p.x(), p.y())];
  return map_grid_value == 0;
}

void RRTPlanner::buildMapImage()
{
  // TODO: Fill out this function to create a cv::Mat object from nav_msgs::OccupancyGrid message
  if (map_received_) {
    int height = map_grid_->info.height;
  	int width = map_grid_->info.width;
  	map_ = std::unique_ptr<cv::Mat>(new cv::Mat(height, width, CV_8UC3));
	for (int x = 0; x < height ; x++) {
		for (int y = 0; y < width; y++) {
  			int index = RRTPlanner::toIndex(x, y);
            int8_t value = map_grid_->data[index];
            cv::Vec3b pixel;
            if (value == 0) {
             pixel = cv::Vec3b(255, 255, 255);
            } else {
             pixel = cv::Vec3b(0, 0, 0);
            }
  			// mirror along the horizontal axis since occupancyGrid starts from top to bottom, left to right and mat is completely opposite
  			map_->at<cv::Vec3b>(height - x - 1, y) = pixel;
		}
	}
  	ROS_INFO("Displaying map");
    drawGoalInitPose();
    drawTreePoints();
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

void RRTPlanner::createTree(int max_iterations, int step_size, int threshold_distance)
{
  ROS_INFO("Creating tree");
  // create and add the starting node in the empty tree
  Node start_node(init_pose_, 0);
  tree_.push_back(start_node);

  for (int i = 0; i < max_iterations; i++) {
    ROS_INFO("Creating a random point");
    Point2D random_point = sample_random_point();
    Node nearest_node = find_nearest_node_in_tree(random_point);
    ROS_INFO_STREAM("Nearest node: "<< nearest_node.position().x() << "," << nearest_node.position().y());
  }
}

void RRTPlanner::drawTreePoints()
{
  // skip the starting node, which is already drawn
  for (int i = 1; i < tree_.size(); i++) {
    ROS_INFO("Drawing a tree");
    drawCircle(tree_[i].position(), 10, cv::Scalar(255, 0, 0));
  }
}


Point2D RRTPlanner::sample_random_point()
{
  nav_msgs::MapMetaData map_meta_data = map_grid_->info;

  int height = map_meta_data.height;
  int width = map_meta_data.width;
  std::random_device rd;
  std::mt19937 gen(rd());
  // distribution object along height axis
  std::uniform_int_distribution<> randomY(0, height - 1);
  std::uniform_int_distribution<> randomX(0, width - 1);
  // assign a random point in the map
  Point2D random_point(randomX(gen), randomY(gen));
  // check if the location is a wall
  while (!isPointUnoccupied(random_point)) {
    random_point.x(randomX(gen));
    random_point.y(randomY(gen));
  }
  ROS_INFO_STREAM("random point: "<< random_point.x() << "," << random_point.y());
  return random_point;
}

/**
* finds the nearest existing node in the tree to the given random point (shortest euclidean distance)
* @param p_random: a randomly selected point in the state space
* @return: returns a node in the tree that is closest to p_random
*/
Node RRTPlanner::find_nearest_node_in_tree(Point2D p_random)
{
   // if only the starting node exists in the tree, return the starting node
   if (tree_.size() == 1) {
     return tree_[0];
   }
   float shortest_distance = std::numeric_limits<float>::max();
   Node closest_node;
   for (int i = 0; i < tree_.size(); i++) {
     Node existing_node = tree_[i];
     Point2D& existing_node_position = existing_node.position();

     float distance = euclideanDistance(existing_node_position, p_random);
     if (distance < shortest_distance) {
       shortest_distance = distance;
       closest_node = tree_[i];
     }
   }
   return closest_node;
}

float RRTPlanner::euclideanDistance(Point2D p1, Point2D p2)
{
   float dy = p1.y() - p2.y();
   float dx = p1.x() - p2.x();
   return std::sqrt(dy * dy + dx * dx);
}


/**
 * adds a new, non-colliding node starting from the nearest existing node (p_nearest) in the direction of the random point,
 * and at a distance of step_size away
 */
Point2D RRTPlanner::grow_to_random_point(Point2D p_nearest, Point2D p_random)
{
  return Point2D();
}

bool RRTPlanner::isValidPath()
{
  return true;
}

}  // namespace rrt_planner
