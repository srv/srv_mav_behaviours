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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <srv_mav_behaviours/mission_managerConfig.h>

#include <srv_mav_behaviours/StartSweep.h>
#include <srv_mav_behaviours/StopSweep.h>
#include <srv_mav_behaviours/PauseSweep.h>
#include <srv_mav_behaviours/ResumeSweep.h>
#include <srv_mav_behaviours/RequestControl.h>
#include <srv_mav_behaviours/GiveUpControl.h>
#include <srv_mav_control/EnablePositionControl.h>

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
  ros::Subscriber pose_subs_;

  ros::Timer timer_;

  dynamic_reconfigure::Server<srv_mav_behaviours::mission_managerConfig> reconfigure_server_;

  // Params
  double min_height, max_height;
  double sweep_y_size, sweep_z_size, sweep_y_increment, sweep_z_increment, sweep_WP_error;

  // Global variables
  double current_x, current_y, current_z, current_yaw;

  bool pose_received;

  bool position_control_granted, position_controllers_enabled;

  bool performing_sweep;
  double WP_x, WP_y, WP_z;
  double initial_yaw_sweep;
  double total_y_displacement;
  double final_z_sweep;
  int sweep_state;
  int sweep_status;

  // Services
  ros::ServiceClient request_control_client_, give_up_control_client_, enable_position_control_client_;
  ros::ServiceServer start_sweep_srv_, stop_sweep_srv_, pause_sweep_srv_, resume_sweep_srv_;

  // Reconfigure
  void dynReconfig(srv_mav_behaviours::mission_managerConfig &config, uint32_t level);

  // Services
  bool startSweep(srv_mav_behaviours::StartSweep::Request &req, srv_mav_behaviours::StartSweep::Response &res);
  bool stopSweep(srv_mav_behaviours::StopSweep::Request &req, srv_mav_behaviours::StopSweep::Response &res);
  bool pauseSweep(srv_mav_behaviours::PauseSweep::Request &req, srv_mav_behaviours::PauseSweep::Response &res);
  bool resumeSweep(srv_mav_behaviours::ResumeSweep::Request &req, srv_mav_behaviours::ResumeSweep::Response &res);

  // Callbacks
  void poseClb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
  void timerClb(const ros::TimerEvent& event);

  // Other methods
  void checkParameters();
  void performSweep();

};

}  // namespace srv_mav_behaviours

#endif // INCLUDE_SRV_MAV_BEHAVIOURS_MISSION_MANAGER_H
