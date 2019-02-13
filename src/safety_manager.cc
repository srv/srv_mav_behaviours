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

  configure();
}

SafetyManager::~SafetyManager(){
}

void SafetyManager::configure(){
  
  double frequency;
  nh_.param("frequency", frequency, 50.0);
  ROS_INFO("Frequency: %2.2f", frequency);

  nh_.param("max_speed_xy", max_speed_xy, 1.0);
  ROS_INFO("Max_speed_xy: %2.2f", max_speed_xy);

  nh_.param("max_speed_z", max_speed_z, 0.7);
  ROS_INFO("Max_speed_z: %2.2f", max_speed_z);

  nh_.param("min_distance_wall", min_distance_wall, 1.2); // minimum distance allowed from walls
  ROS_INFO("Min_distance_wall: %2.2f", min_distance_wall);

  nh_.param("attenuation_distance_wall", attenuation_distance_wall, 2.0); // distance from the wall in meters to start attenuating the speed
  ROS_INFO("Attenuation_distance_wall: %2.2f", attenuation_distance_wall);

  nh_.param("scan_degrees_for_attenuation", scan_degrees_for_attenuation, 40.0); // angular sector of the laser scan considered when computing the attenuation (in degrees)
  ROS_INFO("scan_degrees_for_attenuation: %2.2f", scan_degrees_for_attenuation);

  nh_.param("K_wall_repulsion", K_wall_repulsion, 1.0); // speed for repulsion after penetrating 1m in the forbidden area
  ROS_INFO("K_wall_repulsion: %2.2f", K_wall_repulsion);

  desired_vel_received = false;

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist_out", 1);

  // Subscribers
  user_twist_subs_ = nh_.subscribe("user_twist", 1, &SafetyManager::userTwistClb, this);
  mission_twist_subs_ = nh_.subscribe("laser_scan", 1, &SafetyManager::laserScanClb, this);

  // Advertising Services

  //Service clients

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &SafetyManager::timerClb, this);
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
    half_scans_attenuation = round((scan_degrees_for_attenuation / laser_angle_incr) / 2.0);
  }

  laser_scan_received = true;

}

void SafetyManager::timerClb(const ros::TimerEvent& event){

  if(!(desired_vel_received && laser_scan_received)) return;

  // get the desired command

  double desired_vx, desired_vy, desired_vz, desired_vyaw;

  desired_vx = user_desired_vel.linear.x;
  desired_vy = user_desired_vel.linear.y;
  desired_vz = user_desired_vel.linear.z;
  desired_vyaw = user_desired_vel.angular.z;

  //TODO: if user_desired_vel is 0 and the positionCtrl_vel is not 0, then use the last as desired velocity

  // attenuate the desired command with the proximity of obstacles

  attenuateXYProximity(desired_vx, desired_vy);

  // compute the repulsions from the surrounding obstacles

  double vx_rep, vy_rep;
  computeXYRepulsion(vx_rep, vy_rep);

  // compute final velocity command

  double final_vx, final_vy, final_vz, final_vyaw;
  final_vx = desired_vx + vx_rep;
  final_vy = desired_vy + vy_rep;
  final_vz = desired_vz;
  final_vyaw = desired_vyaw;

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

  float min_range = laser_scan.ranges[initial_index];

  for (int i = initial_index + 1; i <= final_index; i++){

    if (laser_scan.ranges[i] < min_range){

      min_range = laser_scan.ranges[i];

    }

  }

  double attenuation = std::min(1.0, std::max(0.0, min_range - min_distance_wall) / (attenuation_distance_wall - min_distance_wall));

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

}  // namespace srv_mav_behaviours
