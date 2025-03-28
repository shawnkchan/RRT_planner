#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

namespace rrt_planner
{
/**
 * A utility class to represent a 2D point
 */
class Point2D
{
public:
  Point2D(): x_(0), y_(0) {}
  Point2D(int x, int y): x_(x), y_(y) {}

  int x() const
  {
    return x_;
  }

  int y() const
  {
    return y_;
  }

  void x(int x)
  {
    x_ = x;
  }

  void y(int y)
  {
    y_ = y;
  }

private:
  int x_;
  int y_;
};

/**
* Utility class representing a node in the RRT Tree
*/

class Node
{
public:
  Node(): position_(Point2D()), parent_index_(0) {}
  Node(Point2D& position, int parent_index): position_(position), parent_index_(parent_index) {}

  Point2D& position()
  {
    return position_;
  }

  int parent_index()
  {
    return parent_index_;
  }

  void set_parent_index(int parent_index)
  {
    parent_index_ = parent_index;
  }

private:
  Point2D position_;
  int parent_index_;
};

/**
* RRT Algorithm class
RRT(start, goal, max_iterations, step_size):

  tree = {start}

  for i = 1 to max_iterations:

    rand_point = sample_random_point()

    nearest_node = find_nearest_node(tree, rand_point)

    new_node = grow_to_random_point(nearest_node, rand_point, step_size)

    if is_collision_free(nearest_node, new_node):
      add new_node to tree with edge from nearest_node

      if distance(new_node, goal) < threshold:
        return extract_path(tree, start, new_node)

  return failure (no path found)
*/

/**
 * Main class which implements the RRT algorithm
 */
class RRTPlanner
{
public:
  explicit RRTPlanner(ros::NodeHandle *);

  ~RRTPlanner() = default;

  /**
   * Given a map, the initial pose, and the goal, this function will plan
   * a collision-free path through the map from the initial pose to the goal
   * using the RRT algorithm
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void plan();

  /**
   * Callback for map subscriber
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);

  /**
   * Callback for initial pose subscriber
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);

  /**
   * Callback for goal subscriber
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

private:
  /**
   * Publishes the path calculated by RRT as a nav_msgs::Path msg
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void publishPath();

  /**
   * Utility function to check if a given point is free/occupied in the map
   * @param p: point in the map
   * @return boolean true if point is unoccupied, false if occupied
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  bool isPointUnoccupied(const Point2D & p);

  /**
   * Utility function to build a CV::Mat from a nav_msgs::OccupancyGrid for display
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void buildMapImage();

  /**
   * Utility function to draw initial pose and goal pose on the map image
   *
   * THE CANDIDATE IS REQUIRED TO IMPLEMENT THE LOGIC IN THIS FUNCTION
   */
  void drawGoalInitPose();

  /**
   * Utility function to display the CV::Mat map image
   * @param delay
   */
  void displayMapImage(int delay = 1);

  /**
   * Utility function to draw a circle on the map
   * @param p: center point of the circle
   * @param radius: radius of the circle
   * @param color: color of the circle
   */
  void drawCircle(Point2D & p, int radius, const cv::Scalar & color);

  /**
   * Utility function to draw a line on the map
   * @param p1: starting point of the line
   * @param p2: end point of the line
   * @param color: color of the line
   * @param thickness: thickness of the line
   */
  void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);

  /**
   * Utility function to convert a Point2D object to a geometry_msgs::PoseStamped object
   * @return corresponding geometry_msgs::PoseStamped object
   */
  inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

  /**
   * Utility function to convert a geometry_msgs::PoseStamped object to a Point2D object
   */
  inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

  /**
   * Utility function to convert (x, y) matrix coordinate to corresponding vector coordinate
   */
  inline int toIndex(int, int);

  /**
   * returns a random point in the state space that is free
   */
  Point2D sample_random_point();

   /**
   * finds the nearest existing node in the tree to the given random point
   * @param p_random: Point2D object of a random point in the C space
   * @returns index position within the tree array of the nearest existing node
   */
  int find_nearest_node_index_in_tree(Point2D p_random);

  /**
* Utility function to compute euclidean distance betweeen two points
*/
  float euclideanDistance(Point2D p1, Point2D p2);

  /**
   * adds a new node starting from the nearest existing node in the direction of the random point,
   * and at a distance of step_size away
   */
  Point2D grow_to_random_point(Point2D p_nearest, Point2D p_random, int step_size);

  /**
   * checks if the new node is in an occupied spot or if the path taken collides with an occupied spot
   */
  bool isValidPath(Point2D p_nearest, Point2D p_new);

  /**
  * Uses the RRT algorithm to create a tree from start_ point to goal_ point
  */
  void createTree(int max_iterations, int step_size, int threshold_distance);

  /**
  * Utility function to draw points from the Tree array
  */
  void drawTreePoints();

  /**
  * Utility function to draw points from the Path indices array
  */
  void drawPathPoints();

  /**
  * Utility function that returns a deque of indices detailing the path taken from the init_pose_ to end_node
  */
  std::deque<int> getPathIndices(int current_index);

  /**
  * Utility function that takes deque of indices and converts them to a Path message
  */
  nav_msgs::Path getPath(std::deque<int> pathIndices);

  /**
  * ROS parameters
  */
  int max_iterations_;
  int step_size_;
  int threshold_distance_;
  int draw_tree_delay_;
  bool enable_visualization_;
  std::string map_topic_;
  std::string init_pose_topic_;
  std::string goal_topic_;
  std::string path_topic_;

  ros::NodeHandle * nh_;
  ros::NodeHandle private_nh_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  Point2D init_pose_;

  bool goal_received_;
  Point2D goal_;

  std::vector<Node> tree_;
  nav_msgs::Path path_;
  std::deque<int> path_indices_;
  bool path_found_;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
};

}

#endif  // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
