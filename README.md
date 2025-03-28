# RRT Planner (ROS1)

A Rapidly-exploring Random Tree (RRT) implementation in ROS1 for planning a collision-free path from a start position to a goal position in a 2D occupancy grid map.

---

## Features

- Publishes planned path using `nav_msgs/Path`
- Visualization of tree growth and path in OpenCV window
- Map loading via `map_server`
- Customizable parameters via YAML config
- Easily integrates with RViz for visualization

---

## Prerequisites

- ROS1 (Melodic or Noetic recommended)
- `map_server`, `rviz`, `cv_bridge`, `OpenCV`
- Properly sourced ROS workspace with the `rrt_planner` package

---

## How to Run

### 1. Build the workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch the system

```bash
roslaunch rrt_planner rrt_planner.launch
```

This will:

- Start `map_server` to publish the map
- Start `rviz` with the given config
- Start `rrt_planner_node` to begin planning

### 3. Provide Map File

Make sure your `map.yaml` file and corresponding image (PGM/PNG) are correctly placed under `cfg/`. The launch file will load it automatically.

---

## Providing Start & Goal

Use RViz tools:

1. Click `2D Pose Estimate` to set the **start position**
2. Click `2D Nav Goal` to set the **goal position**

---

## Configuration

All tunable parameters can be set in `cfg/config.yaml`:

```yaml
map_topic: "/map"
init_pose_topic: "/initialpose"
goal_topic: "/move_base_simple/goal"
path_topic: "/path"

max_iterations: 2000
step_size: 30
threshold_distance: 40
draw_tree_delay: 100
enable_visualization: true
```

You can adjust these values to influence the RRT behavior and visualization.

---

## Visualization

- If `enable_visualization: true`, tree growth and path are shown in an OpenCV window.
- Tree is drawn in **blue**, path in **orange**, goal in **red**, start in **green**.

---

## Topics

- **Subscribed:**

    - `/map` (`nav_msgs/OccupancyGrid`)
    - `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
    - `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)

- **Published:**

    - `/path` (`nav_msgs/Path`)

---

## Debugging Tips

- Use `rqt_graph` to inspect topic connections
- Add `ROS_INFO_STREAM` logs for runtime data
- Confirm `map_server` is correctly loading the map

---

## Author

*Shawn Chan*

