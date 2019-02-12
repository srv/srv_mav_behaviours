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

  nh_.param("max_speed_xy", max_speed_xy, 0.6);
  ROS_INFO("Max_speed_xy: %2.2f", max_speed_xy);

  nh_.param("max_speed_z", max_speed_z, 0.3);
  ROS_INFO("Max_speed_z: %2.2f", max_speed_z);

  nh_.param("min_distance_wall", min_distance_wall, 1.2);
  ROS_INFO("Min_distance_wall: %2.2f", min_distance_wall);

  nh_.param("K_proximity_attenuation", K_proximity_attenuation, 2.0); // attenuation factor at 1m from the frontier, with regards to the max_speed
  ROS_INFO("K_proximity_attenuation: %2.2f", K_proximity_attenuation);

  nh_.param("K_wall_repulsion", K_wall_repulsion, 1.0);
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
  laser_scan_received = true;

}

void SafetyManager::timerClb(const ros::TimerEvent& event){

  if(!(desired_vel_received && laser_scan_received)) return;

  double final_vx, final_vy, final_vz, final_vyaw;

  final_vx = user_desired_vel.linear.x;
  final_vy = user_desired_vel.linear.y;
  final_vz = user_desired_vel.linear.z;
  final_vyaw = user_desired_vel.angular.z;

  double vx_rep, vy_rep;
  computeXYRepulsion(vx_rep, vy_rep);

  final_vx = final_vx - vx_rep;
  final_vy = final_vy - vy_rep;

  geometry_msgs::TwistPtr final_twist(new geometry_msgs::Twist);
  final_twist->linear.x = final_vx;
  final_twist->linear.y = final_vy;
  final_twist->linear.z = final_vz;
  final_twist->angular.z = final_vyaw;

  twist_pub_.publish(final_twist);

}

void SafetyManager::computeXYRepulsion(double & vx_rep, double & vy_rep){

  vx_rep = vy_rep = 0.0;
  
  int num_ranges = laser_scan.ranges.size();
  float angle_incr = laser_scan.angle_increment;
  float angle = laser_scan.angle_min;

  for(int i = 0; i < num_ranges; i++){

    float range = laser_scan.ranges[i];

    if (!std::isnan(range)){

      if(range < min_distance_wall){

        double repulsion = K_wall_repulsion * max_speed_xy * (min_distance_wall-range);

        vx_rep += repulsion * cos(angle + M_PI);
        vy_rep += repulsion * sin(angle + M_PI);

      }

    }

    angle += angle_incr;

  }

}

}  // namespace srv_mav_behaviours
