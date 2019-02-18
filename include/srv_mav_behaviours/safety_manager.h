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

#ifndef INCLUDE_SRV_MAV_BEHAVIOURS_SAFETY_MANAGER_H
#define INCLUDE_SRV_MAV_BEHAVIOURS_SAFETY_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


namespace srv_mav_behaviours {

class SafetyManager {
 public:
  explicit SafetyManager(const ros::NodeHandle& nh);
  virtual ~SafetyManager();

  void configure();

 private:
  // ROS variables
  ros::NodeHandle nh_;

  ros::Subscriber user_twist_subs_, mission_twist_subs_;
  ros::Subscriber laser_scan_subs_;

  ros::Publisher twist_pub_;
  ros::Publisher laser_pub_;

  ros::Timer timer_;

  // Params
  double min_distance_wall, max_height;
  double attenuation_distance_wall;
  double scan_degrees_for_attenuation;
  double K_wall_repulsion;
  double max_speed_xy, max_speed_z;
  int laser_filter_size;

  // Global variables
  geometry_msgs::Twist user_desired_vel;
  bool desired_vel_received;
  sensor_msgs::LaserScan laser_scan;
  bool laser_scan_received;
  int laser_num_ranges;
  float laser_angle_incr;
  float laser_angle_min;
  int half_scans_attenuation;
  int laser_half_filter;

  // Services

  // Callbacks
  void userTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg);
  void laserScanClb(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg);
  void timerClb(const ros::TimerEvent& event);

  // Other methods
  void attenuateXYProximity(double & x_vel, double & y_vel);
  void computeXYRepulsion(double & vx_rep, double & vy_rep);

};

}  // namespace srv_mav_behaviours

#endif // INCLUDE_SRV_MAV_BEHAVIOURS_SAFETY_MANAGER_H
