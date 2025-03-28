#include "ros/ros.h"
#include "rrt_planner/rrt_planner.h"
#include <nav_msgs/Path.h>
#include <cmath>
#include <deque>

namespace rrt_planner
{
RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false),
  path_found_(false)
{

  // TODO: Fill out this function to:
  // Get map and path topics from parameter server
  private_nh_.param("max_iterations", max_iterations_, 2000);
  private_nh_.param("step_size", step_size_, 30);
  private_nh_.param("threshold_distance", threshold_distance_, 40);
  private_nh_.param("draw_tree_delay", draw_tree_delay_, 100);
  private_nh_.param("enable_visualization", enable_visualization_, true);
  private_nh_.param<std::string>("map_topic", map_topic_, "/map");
  private_nh_.param<std::string>("init_pose_topic", init_pose_topic_, "/initialpose");
  private_nh_.param<std::string>("goal_topic", goal_topic_, "/move_base_simple/goal");
  private_nh_.param<std::string>("path_topic", path_topic_, "/path");

  // Subscribe to map topic
  map_sub_ = nh_->subscribe(map_topic_, 10, &RRTPlanner::mapCallback, this);
  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe(init_pose_topic_, 10, &RRTPlanner::initPoseCallback, this);
  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe(goal_topic_, 10, &RRTPlanner::goalCallback, this);
  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic_, 10);
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
  drawCircle(goal_, 5, cv::Scalar(0, 0, 255));
  drawCircle(init_pose_, 5, cv::Scalar(0, 255, 0));
}

void RRTPlanner::plan()
{
  // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
  //       path through the map starting from the initial pose and ending at the goal pose
  ROS_INFO("Starting Planning");
  ros::Rate loop_rate(5);
  bool planning_complete = false;
  while (ros::ok())
    {
    	if (map_received_ && goal_received_ && init_pose_received_) {
			buildMapImage();
            if (!planning_complete) {
              createTree(max_iterations_, step_size_, threshold_distance_);
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
  path_pub_.publish(path_);
}

bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // TODO: Fill out this function to check if a given point is occupied/free in the map
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
    drawGoalInitPose();
    drawTreePoints();
    if (path_found_) {
      drawPathPoints();
    }
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
  // create and add the starting node in the empty tree
  Node start_node(init_pose_, 0);
  tree_.push_back(start_node);

  for (int i = 0; i < max_iterations; i++) {
    Point2D random_point = sample_random_point();
    int nearest_node_index = find_nearest_node_index_in_tree(random_point);
    Node nearest_node = tree_[nearest_node_index];
    Point2D new_point = grow_to_random_point(nearest_node.position(), random_point, step_size);
    // if the new point is in an unoccupied spot and if the path from nearest point to new point has no collisions, then add it to the tree
    if (isPointUnoccupied(new_point) && isValidPath(nearest_node.position(), new_point)) {
      // make new_point into a node and add to tree array
      Node new_node(new_point, nearest_node_index);
      tree_.push_back(new_node);

      drawCircle(new_point, 5, cv::Scalar(255, 0, 0));
      drawLine(new_point, nearest_node.position(), cv::Scalar(255, 0, 0));
      if (enable_visualization_) {
        displayMapImage(100);
      }
      // if the newly added point is within the threshold distance, then terminate
      if (euclideanDistance(new_point, goal_) <= threshold_distance && isValidPath(new_point, goal_)) {
             drawLine(new_point, goal_, cv::Scalar(255, 0, 0));
             displayMapImage(draw_tree_delay_);
             int current_index = tree_.size() - 1;
             path_indices_ = getPathIndices(current_index);
             path_ = getPath(path_indices_);
             publishPath();
             path_found_ = true;
             break;
      }
    }
  }
  if (!path_found_) {
    ROS_INFO("No valid path found");
  } else {
    ROS_INFO("Path found");
  }
}

void RRTPlanner::drawTreePoints()
{
  // skip the starting node, which is already drawn
  for (int i = 1; i < tree_.size(); i++) {
    drawCircle(tree_[i].position(), 5, cv::Scalar(255, 0, 0));
    int parent_index = tree_[i].parent_index();
    Node parent_node = tree_[parent_index];
    drawLine(tree_[i].position(), parent_node.position(), cv::Scalar(255, 0, 0));
  }
}

void RRTPlanner::drawPathPoints()
{
  for (int i = 1; i < path_indices_.size(); i++) {
    int tree_index = path_indices_[i];
    drawCircle(tree_[tree_index].position(), 5, cv::Scalar(0, 165, 255));
    int parent_index = tree_[tree_index].parent_index();
    Node parent_node = tree_[parent_index];
    drawLine(tree_[tree_index].position(), parent_node.position(), cv::Scalar(0, 165, 255));
  }
  drawLine(tree_[tree_.size() - 1].position(), goal_, cv::Scalar(0, 165, 255));
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
  return random_point;
}

/**
* finds the nearest existing node in the tree to the given random point (shortest euclidean distance)
* @param p_random: a randomly selected point in the state space
* @return: returns a node in the tree that is closest to p_random
*/
int RRTPlanner::find_nearest_node_index_in_tree(Point2D p_random)
{
   // if only the starting node exists in the tree, return the starting node
   if (tree_.size() == 1) {
     return 0;
   }
   float shortest_distance = std::numeric_limits<float>::max();
   int closest_node_index;
   // check through all tree nodes and return which node is closest
   for (int i = 0; i < tree_.size(); i++) {
     Node existing_node = tree_[i];
     Point2D& existing_node_position = existing_node.position();

     float distance = euclideanDistance(existing_node_position, p_random);
     if (distance < shortest_distance) {
       shortest_distance = distance;
       closest_node_index = i;
     }
   }
   return closest_node_index;
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
Point2D RRTPlanner::grow_to_random_point(Point2D p_nearest, Point2D p_random, int step_size)
{
  // NOTE: if dist(p_random, p_nearest) < step_size, then just place at p_random.
  // p_random is guaranteed not to be a collision
  float distance = euclideanDistance(p_nearest, p_random);
  if (distance < step_size) {
    return p_random;
  }
  float scale = step_size / distance;
  int dx = p_random.x() - p_nearest.x();
  int dy = p_random.y() - p_nearest.y();
  int new_x = std::round(dx * scale + p_nearest.x());
  int new_y = std::round(dy * scale + p_nearest.y());
  return Point2D(new_x, new_y);
}

bool RRTPlanner::isValidPath(Point2D p_nearest, Point2D p_new)
{
  int dx = p_new.x() - p_nearest.x();
  int dy = p_new.y() - p_nearest.y();
  int steps = std::max(std::abs(dx), std::abs(dy));
  for (int i = 0; i < steps; i++) {
    float t = float(i) / steps;
    int x = std::round(p_nearest.x() + t * dx);
    int y = std::round(p_nearest.y() + t * dy);
    Point2D path_point(x, y);
    if (!isPointUnoccupied(path_point)) {
      return false;
    }
  }
  return true;
}

std::deque<int> RRTPlanner::getPathIndices(int current_index)
{
  std::deque<int> path;

  while (true) {
    path.push_front(current_index);
    Node current_node = tree_[current_index];
    int parent_node_index = current_node.parent_index();
    if (parent_node_index == current_index) {
      break;
    }
    current_index = parent_node_index;
  }
  return path;
}

nav_msgs::Path RRTPlanner::getPath(std::deque<int> path_indices)
{
  nav_msgs::Path path;
  std::vector<geometry_msgs::PoseStamped> poses;

  for (int i = 0; i < path_indices.size(); i++) {
    int tree_index = path_indices[i];
    Node node = tree_[tree_index];
    Point2D node_position = node.position();
    geometry_msgs::PoseStamped node_pose = pointToPose(node_position);
    poses.push_back(node_pose);
  }
  poses.push_back(pointToPose(goal_));
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses = poses;
  return path;
}



}  // namespace rrt_planner
