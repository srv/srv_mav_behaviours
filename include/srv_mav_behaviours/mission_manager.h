/*
* This file is part of srv_mav_behaviours.
*
* Copyright (C) 2018 Francisco bonnin-Pascual <xisco.bonnin@uib.es> (University of the Balearic Islands)
*
* srv_mav_behaviours is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* srv_mav_behaviours is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with srv_mav_behaviours. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_SRV_MAV_BEHAVIOURS_MISSION_MANAGER_H
#define INCLUDE_SRV_MAV_BEHAVIOURS_MISSION_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <srv_mav_behaviours/mission_managerConfig.h>

#include <std_srvs/Empty.h>
#include <srv_mav_behaviours/StartSweep.h>
#include <srv_mav_behaviours/StartVerticalInspection.h>
#include <srv_mav_behaviours/RequestControl.h>
#include <srv_mav_behaviours/GiveUpControl.h>
#include <srv_mav_control/EnablePositionControl.h>
#include <srv_mav_behaviours/GoToPoint.h>
#include <srv_mav_behaviours/SavePoint.h>

namespace srv_mav_behaviours {

class MissionManager {
 public:
  explicit MissionManager(const ros::NodeHandle& nh);
  virtual ~MissionManager();

  void configure();

 private:
  // ROS variables
  ros::NodeHandle nh_;

  ros::Publisher pose_pub_;
  ros::Publisher mission_path_pub_;
  ros::Publisher WP_path_pub_;
  ros::Subscriber odom_subs_;
  ros::Subscriber min_distance_left_subs_, min_distance_right_subs_, min_distance_up_subs_, min_distance_down_subs_;

  ros::Timer timer_;

  dynamic_reconfigure::Server<srv_mav_behaviours::mission_managerConfig> reconfigure_server_;

  // Params
  double min_height, max_height;
  double WP_tolerance;
  double sweep_min_lateral_dist;
  double vinspection_min_ceiling_dist;
  bool follow_trajectory;
  double follow_trajectory_delta; // maximum distance to the VTP
  double follow_trajectory_lambda; // maximum distance to the path

  // Global variables
  std::string world_frame;
  double current_x, current_y, current_z, current_yaw;

  bool odom_received;

  double min_dist_left, min_dist_right, min_dist_up, min_dist_down;

  bool position_controllers_enabled;

  double WP_x, WP_y, WP_z;

  double pausedMission_WP_x, pausedMission_WP_y, pausedMission_WP_z;
  double pausedMission_yaw;

  double sweep_y_size, sweep_z_size, sweep_y_increment, sweep_z_increment;
  bool sweep_wall_to_wall;
  bool performing_sweep;
  double initial_yaw_sweep;
  double sweep_y_accumulated;
  double final_z_sweep;
  int sweep_state;
  int sweep_status;
  bool sweep_reaching_end;

  double vinspection_y_size, vinspection_z_size, vinspection_z_increment;
  bool vinspection_to_ceiling;
  bool performing_vinspection;
  double initial_yaw_vinspection;
  double vinspection_z_accumulated;
  double final_z_vinspection;
  int vinspection_state;
  int vinspection_status;
  bool vinspection_reaching_end;

  bool publish_mission_path;
  nav_msgs::PathPtr mission_path;
  nav_msgs::PathPtr WP_path;

  double home_x, home_y, home_z;

  std::vector<double> saved_positions_x;
  std::vector<double> saved_positions_y;
  std::vector<double> saved_positions_z;
  std::vector<std::string> saved_positions_description;

  // Services
  ros::ServiceClient request_control_client_, give_up_control_client_, enable_position_control_client_;
  ros::ServiceServer start_sweep_srv_, stop_sweep_srv_, pause_sweep_srv_, resume_sweep_srv_;
  ros::ServiceServer start_vertical_inspection_srv_, stop_vertical_inspection_srv_, pause_vertical_inspection_srv_, resume_vertical_inspection_srv_;
  ros::ServiceServer hover_srv_;
  ros::ServiceServer go_home_srv_, set_home_srv_;
  ros::ServiceServer save_point_srv_, go_to_point_srv_;

  // Reconfigure
  void dynReconfig(srv_mav_behaviours::mission_managerConfig &config, uint32_t level);

  // Services
  bool startSweep(srv_mav_behaviours::StartSweep::Request &req, srv_mav_behaviours::StartSweep::Response &res);
  bool stopSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool pauseSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool resumeSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool startVerticalInspection(srv_mav_behaviours::StartVerticalInspection::Request &req, srv_mav_behaviours::StartVerticalInspection::Response &res);
  bool stopVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool pauseVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool resumeVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool hover(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool setHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool savePoint(srv_mav_behaviours::SavePoint::Request &req, srv_mav_behaviours::SavePoint::Response &res);
  bool goToPoint(srv_mav_behaviours::GoToPoint::Request &req, srv_mav_behaviours::GoToPoint::Response &res);

  // Callbacks
  void odomClb(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void minDistanceLeftClb(const sensor_msgs::Range::ConstPtr& range_msg);
  void minDistanceRightClb(const sensor_msgs::Range::ConstPtr& range_msg);
  void minDistanceUpClb(const sensor_msgs::Range::ConstPtr& range_msg);
  void minDistanceDownClb(const sensor_msgs::Range::ConstPtr& range_msg);
  void timerClb(const ros::TimerEvent& event);

  // Other methods
  void checkParameters();
  void performSweep();
  void performVerticalInspection();
  void performHovering();
  void performGoHome();
  void performGoToPoint(double point_x, double point_y, double point_z);

  void publishMissionPath();
  void clearMissionPath();
  void createSweepingPath();
  void recomputeSweepingPath();
  void createVerticalInspectionPath();
  void recomputeVerticalInspectionPath();

  void publishWPPath();
  void newWPPath();
  void addWP2WPPath(double x, double y, double z);
  void removeLastWPPath();

};

}  // namespace srv_mav_behaviours

#endif // INCLUDE_SRV_MAV_BEHAVIOURS_MISSION_MANAGER_H
