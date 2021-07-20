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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <srv_mav_msgs/MAVVerticalState.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <srv_mav_behaviours/safety_managerConfig.h>

#include <srv_mav_behaviours/RequestControl.h>
#include <srv_mav_behaviours/GiveUpControl.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGBA      Point;
typedef pcl::PointCloud<Point> PointCloud;

namespace srv_mav_behaviours {

class SafetyManager {
 public:
  explicit SafetyManager(const ros::NodeHandle& nh);
  virtual ~SafetyManager();

  void configure();

 private:
  // ROS variables
  ros::NodeHandle nh_;

  ros::Subscriber user_twist_subs_, position_ctrl_twist_subs_;
  ros::Subscriber height_subs_, ceiling_distance_subs_, ground_distance_subs_;
  ros::Subscriber point_cloud_subs_;
  ros::Subscriber flight_status_subs_;

  ros::Publisher twist_pub_;
  ros::Publisher mean_dist_front_pub_;
  ros::Publisher min_dist_front_pub_, min_dist_left_pub_, min_dist_right_pub_, min_dist_back_pub_;
  ros::Publisher min_dist_front_left_pub_, min_dist_front_right_pub_, min_dist_back_left_pub_, min_dist_back_right_pub_;
  ros::Publisher main_ori_pub_;
  ros::Publisher mean_ori_pub_;

  ros::Publisher point_cloud_pub_;

  ros::Timer timer_;

  dynamic_reconfigure::Server<srv_mav_behaviours::safety_managerConfig> reconfigure_server_;

  //Services
  ros::ServiceServer request_control_srv_;
  ros::ServiceServer give_up_control_srv_;

  // Params
  double robot_radius;
  double min_distance_wall, min_distance_ceiling, max_height;
  double attenuation_distance_wall, attenuation_distance_ceiling, attenuation_max_height;
  double degrees_for_attenuation;
  double K_wall_repulsion, K_ceiling_repulsion, K_max_height_attraction;
  double max_speed;

  // Global variables
  unsigned int flight_status;
  geometry_msgs::Twist user_desired_vel;
  bool desired_vel_received;
  geometry_msgs::Twist position_ctrl_vel;
  bool position_ctrl_vel_received;
  bool position_control_granted;
  PointCloud point_cloud;
  bool point_cloud_received;
  // int laser_num_ranges;
  // float laser_angle_incr;
  // float laser_angle_min;
  // int half_scans_attenuation;
  // int laser_half_filter;
  // double distance_back;
  // bool distance_back_received;
  double height;
  bool height_received;
  double distance_ceiling;
  bool distance_ceiling_received;
  double distance_ground;
  bool distance_ground_received;

  // Services
  bool requestControl(srv_mav_behaviours::RequestControl::Request &req, srv_mav_behaviours::RequestControl::Response &res);
  bool giveUpControl(srv_mav_behaviours::GiveUpControl::Request &req, srv_mav_behaviours::GiveUpControl::Response &res);

  // Reconfigure
  void dynReconfig(srv_mav_behaviours::safety_managerConfig &config, uint32_t level);

  // Callbacks
  void flightStatusClb(const std_msgs::UInt8::ConstPtr& flight_status_msg);
  void positionCtrlTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg);
  void userTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg);
  void pointCloudClb(const PointCloud::ConstPtr& point_cloud_msg);
  void heightClb(const srv_mav_msgs::MAVVerticalState::ConstPtr& height_msg);
  void ceilingDistanceClb(const sensor_msgs::Range::ConstPtr& ceiling_distance_msg);
  void groundDistanceClb(const sensor_msgs::Range::ConstPtr& ground_distance_msg);
  void timerClb(const ros::TimerEvent& event);

  // Other methods
  void checkParameters();
  void attenuateXYZProximity(double & x_vel, double & y_vel, double & z_vel);
  void computeXYZRepulsion(double & vx_rep, double & vy_rep, double & vz_rep);
  void attenuateZProximity(double & z_vel);
  void computeZRepulsion(double & vz_rep);
  void attenuateZMaxHeight(double & z_vel);
  void computeZAttraction(double & vz_att);
  float getMeanDistanceFront();
  float getMinDistance(int direction);
  double getOrientationFrontMean();
  double getOrientationFrontMain();

};

}  // namespace srv_mav_behaviours

#endif // INCLUDE_SRV_MAV_BEHAVIOURS_SAFETY_MANAGER_H
