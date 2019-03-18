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

#include "srv_mav_behaviours/safety_manager.h"

namespace srv_mav_behaviours{

SafetyManager::SafetyManager(const ros::NodeHandle& nh) :
  nh_(nh)
{

  reconfigure_server_.setCallback(boost::bind(&SafetyManager::dynReconfig, this, _1, _2));
  configure();
}

SafetyManager::~SafetyManager(){
}

void SafetyManager::dynReconfig(srv_mav_behaviours::safety_managerConfig &config, uint32_t level){

  max_speed_xy = config.max_speed_xy;
  max_speed_z = config.max_speed_z;

  laser_filter_size = config.laser_filter_size; // size of the window used to filter the laser scan (mean filter)
  laser_half_filter = laser_filter_size / 2; 
  laser_filter_size = laser_half_filter * 2 + 1; // 11 --> 11; 10 --> 11 (filter size always becomes an odd number)

  min_distance_wall = config.min_distance_wall; // minimum distance allowed from walls
  attenuation_distance_wall = config.attenuation_distance_wall; // distance from the wall in meters to start attenuating the speed
  scan_degrees_for_attenuation = config.scan_degrees_for_attenuation; // angular sector of the laser scan considered when computing the attenuation (in degrees)
  K_wall_repulsion = config.K_wall_repulsion; // speed in m/s for the repulsion after penetrating 1 m in the forbidden area

  max_height = config.max_height;
  attenuation_max_height = config.attenuation_max_height; // height in meters to start attenuating the vertical speed
  K_max_height_attraction = config. K_max_height_attraction; //speed in m/s for the attraction to the ground after trespassing 1 m the maximum height allowed

  if(laser_scan_received){
    half_scans_attenuation = round(scan_degrees_for_attenuation * M_PI / 180.0 / 2.0 / laser_angle_incr);
  }

  checkParameters();

}

void SafetyManager::configure(){
  
  double frequency;
  nh_.param("frequency", frequency, 50.0);
  ROS_INFO("Frequency: %2.2f", frequency);

  nh_.param("max_speed_xy", max_speed_xy, 1.0);
  ROS_INFO("Max_speed_xy: %2.2f", max_speed_xy);

  nh_.param("max_speed_z", max_speed_z, 0.7);
  ROS_INFO("Max_speed_z: %2.2f", max_speed_z);

  nh_.param("laser_filter_size", laser_filter_size, 11); // size of the window used to filter the laser scan (mean filter)
  laser_half_filter = laser_filter_size / 2; 
  laser_filter_size = laser_half_filter * 2 + 1; // 11 --> 11; 10 --> 11 (filter size always becomes an odd number)
  ROS_INFO("Laser_filter_size: %d", laser_filter_size);

  nh_.param("min_distance_wall", min_distance_wall, 1.2); // minimum distance allowed from walls
  ROS_INFO("Min_distance_wall: %2.2f", min_distance_wall);

  nh_.param("attenuation_distance_wall", attenuation_distance_wall, 2.0); // distance from the wall in meters to start attenuating the speed
  ROS_INFO("Attenuation_distance_wall: %2.2f", attenuation_distance_wall);

  nh_.param("scan_degrees_for_attenuation", scan_degrees_for_attenuation, 40.0); // angular sector of the laser scan considered when computing the attenuation (in degrees)
  ROS_INFO("scan_degrees_for_attenuation: %2.2f", scan_degrees_for_attenuation);

  nh_.param("K_wall_repulsion", K_wall_repulsion, 1.0); // speed in m/s for the repulsion after penetrating 1 m in the forbidden area
  ROS_INFO("K_wall_repulsion: %2.2f", K_wall_repulsion);

  nh_.param("max_height", max_height, 4.0);
  ROS_INFO("Max_height: %2.2f", max_height);

  nh_.param("attenuation_max_height", attenuation_max_height, 3.0); // height in meters to start attenuating the positive vertical speed
  ROS_INFO("attenuation_max_height: %2.2f", attenuation_max_height);

  nh_.param("K_max_height_attraction", K_max_height_attraction, 1.0);
  ROS_INFO("K_max_height_attraction: %2.2f", K_max_height_attraction); //speed in m/s for the attraction to the ground after trespassing 1 m the maximum height allowed

  checkParameters();

  desired_vel_received = false;
  position_ctrl_vel_received = false;
  laser_scan_received = false;
  height_received = false;

  allowing_position_ctrl = false;

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist_out", 1);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_obstacles", 1);

  // Subscribers
  user_twist_subs_ = nh_.subscribe("user_twist", 1, &SafetyManager::userTwistClb, this);
  position_ctrl_twist_subs_ = nh_.subscribe("position_ctrl_twist", 1, &SafetyManager::positionCtrlTwistClb, this);
  laser_scan_subs_ = nh_.subscribe("laser_scan", 1, &SafetyManager::laserScanClb, this);
  height_subs_ = nh_.subscribe("height", 1, &SafetyManager::heightClb, this);

  // Advertising Services
  request_control_srv_ = nh_.advertiseService("request_control", &SafetyManager::requestControl, this);

  //Service clients

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &SafetyManager::timerClb, this);
}

void SafetyManager::checkParameters(){

  max_speed_xy = abs(max_speed_xy);
  max_speed_z = abs(max_speed_z);

  min_distance_wall = abs(min_distance_wall);
  attenuation_distance_wall = abs(attenuation_distance_wall);
  K_wall_repulsion = abs(K_wall_repulsion);

  max_height = abs(max_height);
  attenuation_max_height = abs(attenuation_max_height);
  K_max_height_attraction = abs(K_max_height_attraction);

  if(attenuation_distance_wall <= min_distance_wall){
    attenuation_distance_wall = min_distance_wall + 1;
    ROS_WARN("attenuation_distance_wall must be larger than min_distance_wall");
    ROS_WARN("attenuation_distance_wall set to %2.2f", attenuation_distance_wall);
  }

  if(max_height < 2.0){
    max_height = 2.0;
    ROS_WARN("max_height set to %2.2f m", max_height);
  }

  if(attenuation_max_height >= max_height){
    attenuation_max_height = max_height - 1;
    ROS_WARN("attenuation_max_height must be below max_height");
    ROS_WARN("attenuation_max_height set to %2.2f", attenuation_max_height);
  }

}

bool SafetyManager::requestControl(srv_mav_behaviours::RequestControl::Request &req, srv_mav_behaviours::RequestControl::Response &res){

  if(!allowing_position_ctrl){

    allowing_position_ctrl = true;
    res.allowed = true;
    ROS_WARN("Allowing autonomous behaviour");

  }else{
    res.allowed = false;
    ROS_WARN("Autonomous behaviour was already allowed");
  }

  return true;

}

void SafetyManager::positionCtrlTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  position_ctrl_vel = *twist_msg;
  position_ctrl_vel_received = true;

}

void SafetyManager::userTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  user_desired_vel = *twist_msg;
  desired_vel_received = true;

}

void SafetyManager::laserScanClb(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg){

  laser_scan = *laser_scan_msg;

  if(!laser_scan_received){
    laser_num_ranges = laser_scan.ranges.size();
    laser_angle_incr = laser_scan.angle_increment;
    laser_angle_min = laser_scan.angle_min;
    half_scans_attenuation = round(scan_degrees_for_attenuation * M_PI / 180.0 / 2.0 / laser_angle_incr);
  }

  float mean_range = 0.0;
  int good_ranges  = 0;

  for (int i = 0; i <= 2*laser_half_filter; i++){

    float range = laser_scan_msg->ranges[i];

    if(!std::isnan(range)){

      mean_range += range;
      good_ranges ++;

    }

  }

  if(!std::isnan(laser_scan_msg->ranges[laser_half_filter])){ // good_ranges is at least 1

    laser_scan.ranges[laser_half_filter] = mean_range / good_ranges;

  }

  for (int i = laser_half_filter+1; i < laser_num_ranges - laser_half_filter; i++){ 

    float oldRange = laser_scan_msg->ranges[i-laser_half_filter-1];
    float newRange = laser_scan_msg->ranges[i+laser_half_filter];
    float range = laser_scan_msg->ranges[i];

    if(!std::isnan(oldRange)){
      mean_range -= oldRange;
      good_ranges --;
    }

    if(!std::isnan(newRange)){
      mean_range += newRange;
      good_ranges ++;
    }
    if(!std::isnan(range)) laser_scan.ranges[i] = mean_range / good_ranges; // good_ranges is at least 1

  }

  laser_pub_.publish(laser_scan);

  laser_scan_received = true;

}

void SafetyManager::heightClb(const srv_mav_msgs::MAVVerticalState::ConstPtr& height_msg){

  height = height_msg->z;
  height_received = true;

}

void SafetyManager::timerClb(const ros::TimerEvent& event){

  if(!(desired_vel_received && laser_scan_received && height_received)) return;

  // get the desired command

  double desired_vx, desired_vy, desired_vz, desired_vyaw;

  desired_vx = user_desired_vel.linear.x;
  desired_vy = user_desired_vel.linear.y;
  desired_vz = user_desired_vel.linear.z;
  desired_vyaw = user_desired_vel.angular.z;

  if(allowing_position_ctrl){

    if((desired_vx == 0.0) && (desired_vy == 0.0)){

      desired_vx = position_ctrl_vel.linear.x;
      desired_vy = position_ctrl_vel.linear.y;
      desired_vz = position_ctrl_vel.linear.z;

    }else{ // the autonomous behaviour can be stopped sending commands in vX or vY

      allowing_position_ctrl = false;
      ROS_WARN("Stopping autonomous behaviour");

      //stop all the autonomous behaviours
      nh_.setParam("performing_sweep", false);

    }

  }

  //TODO: if user_desired_vel is 0 and the positionCtrl_vel is not 0, then use the last as desired velocity

  // attenuate the desired command in XY with the proximity of obstacles
  attenuateXYProximity(desired_vx, desired_vy);

  //attenuate the desired command in Z with the proximity to the maximum height allowed
  attenuateZMaxHeight(desired_vz);

  // compute the repulsions from the surrounding obstacles
  double vx_rep, vy_rep;
  computeXYRepulsion(vx_rep, vy_rep);

  // compute the attraction to the ground when trespassing the maximum height allowed
  double vz_att;
  computeZAttraction(vz_att);

  // compute final velocity command
  double final_vx, final_vy, final_vz, final_vyaw;
  final_vx = desired_vx + vx_rep;
  final_vy = desired_vy + vy_rep;
  final_vz = desired_vz + vz_att;
  final_vyaw = desired_vyaw;

  // limit with the maximum speed allowed
  if (final_vx > max_speed_xy) final_vx = max_speed_xy;
  else if (final_vx < -max_speed_xy) final_vx = -max_speed_xy;

  if (final_vy > max_speed_xy) final_vy = max_speed_xy;
  else if (final_vy < -max_speed_xy) final_vy = -max_speed_xy;

  if (final_vz > max_speed_z) final_vz = max_speed_z;
  else if (final_vz < -max_speed_z) final_vz = -max_speed_z;

  // publish final velocity command

  geometry_msgs::TwistPtr final_twist(new geometry_msgs::Twist);
  final_twist->linear.x = final_vx;
  final_twist->linear.y = final_vy;
  final_twist->linear.z = final_vz;
  final_twist->angular.z = final_vyaw;

  twist_pub_.publish(final_twist);

}

void SafetyManager::attenuateXYProximity(double & x_vel, double & y_vel){

  double angle = atan2(y_vel, x_vel);

  int index = round((angle - laser_angle_min) / laser_angle_incr);

  int initial_index = std::max(0, index - half_scans_attenuation);
  int final_index = std::min(laser_num_ranges-1, index + half_scans_attenuation);

  float min_range = 100.0;

  for (int i = initial_index; i <= final_index; i++){

    float range = laser_scan.ranges[i];

    if (!std::isnan(range)){

      if (range < min_range){

        min_range = range;

      }
    }

  }

  // Ds = min_range - min_distance_wall                  --> Distance to the stop fence
  // Dsp = std::max(0.0, Ds)                             --> Ds must be positive. If Ds is negative the attenuation is complete
  // Da = attenuation_distance_wall - min_distance_wall  --> Distance from the attenuation fence to the stop fence
  // P = Dsp / Da                                        --> Situation between fences given as a proportion. It P > 1 there is no attenuation
  // attenuation = std::min(1.0, P)

  double attenuation = std::min(1.0, std::max(0.0, min_range - min_distance_wall) / (attenuation_distance_wall - min_distance_wall));

  /*ROS_INFO("min_range: %f", min_range);
  ROS_INFO("Attenuation: %f", attenuation);*/

  //attenuation is in [0.0, 1.0]

  x_vel = x_vel * attenuation;
  y_vel = y_vel * attenuation;

}

void SafetyManager::computeXYRepulsion(double & vx_rep, double & vy_rep){

  vx_rep = vy_rep = 0.0;
  int num_rep = 0;

  float angle = laser_scan.angle_min;

  for(int i = 0; i < laser_num_ranges; i++){

    float range = laser_scan.ranges[i];

    if (!std::isnan(range)){

      if(range < min_distance_wall){

        // Dt = min_distance_wall-range          --> Indicates how much we have trespassed the stop fence. It is always positive.
        // R = K_wall_repulsion * Dt             --> Repulsion speed 
        // repulsion = std::min(max_speed_xy, R) --> Limit repulsion with the maximum speed allowed

        double repulsion = std::min(max_speed_xy, K_wall_repulsion * (min_distance_wall-range));

        // repulsion is in [0.0, max_speed_xy]

        vx_rep += repulsion * cos(angle + M_PI);
        vy_rep += repulsion * sin(angle + M_PI);
        num_rep ++;

      }

    }

    angle += laser_angle_incr;

  }

  if(num_rep > 0){

    vx_rep = vx_rep / num_rep;
    vy_rep = vy_rep / num_rep;

  }

}

void SafetyManager::attenuateZMaxHeight(double & z_vel){

  double attenuation = 1.0;

  if(z_vel > 0.0){ // only  if we want to go higher

    // Ds = max_height - height                  --> Distance to the maximum height
    // Dsp = std::max(0.0, Ds)                   --> Ds must be positive. If Ds is negative the attenuation is complete
    // Da = max_height - attenuation_max_height  --> Distance from the attenuation fence to the stop fence
    // P = Dsp / Da                              --> Situation between fences given as a proportion. It P > 1 there is no attenuation
    // attenuation = std::min(1.0, P)

    attenuation = std::min(1.0, std::max(0.0, max_height - height) / (max_height - attenuation_max_height));

  }

  //attenuation is in [0.0, 1.0]

  z_vel = z_vel * attenuation;

}

void SafetyManager::computeZAttraction(double & vz_att){

  vz_att = 0.0;

  // Dt = max(0.0, height-max_height)      --> Indicates how much we have trespassed the maximum height. A negative value means no attraction.
  // A = K_max_height_attraction * Dt      --> Attraction speed 
  // attraction = std::min(max_speed_z, A) --> Limit repulsion with the maximum speed allowed

  // negative speed to make the MAV descend
  vz_att = -std::min(max_speed_z, K_max_height_attraction * std::max(0.0, height-max_height));

}

}  // namespace srv_mav_behaviours
