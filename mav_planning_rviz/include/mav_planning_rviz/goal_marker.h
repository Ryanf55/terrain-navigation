
#ifndef MAV_PLANNING_RVIZ_GOAL_MARKER_H_
#define MAV_PLANNING_RVIZ_GOAL_MARKER_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <grid_map_msgs/GridMap.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
class GoalMarker {
 public:
  GoalMarker(const ros::NodeHandle &nh);
  virtual ~GoalMarker();

 private:
  Eigen::Vector3d toEigen(const geometry_msgs::Pose &p) {
    Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
    return position;
  }
  void processSetPoseFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void GridmapCallback(const grid_map_msgs::GridMap &msg);

  ros::NodeHandle nh_;
  ros::Subscriber grid_map_sub_;
  interactive_markers::InteractiveMarkerServer marker_server_;
  visualization_msgs::InteractiveMarker set_goal_marker_;

  Eigen::Vector3d goal_pos_{Eigen::Vector3d::Zero()};
  grid_map::GridMap map_;
};

#endif  // MAV_PLANNING_RVIZ_GOAL_MARKER_H_
