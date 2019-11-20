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

  min_distance_ceiling = config.min_distance_ceiling; // minimum distance allowed from the ceiling
  attenuation_distance_ceiling = config.attenuation_distance_ceiling; // distance from the ceiling in meters to start attenuating the speed
  K_ceiling_repulsion = config.K_ceiling_repulsion; // speed in m/s for the repulsion after penetrating 1 m in the forbidden area

  max_height = config.max_height;
  attenuation_max_height = config.attenuation_max_height; // height in meters to start attenuating the vertical speed
  K_max_height_attraction = config. K_max_height_attraction; //speed in m/s for the attraction to the ground after trespassing 1 m the maximum height allowed

  if(laser_scan_received){
    half_scans_attenuation = round(scan_degrees_for_attenuation * M_PI / 180.0 / 2.0 / laser_angle_incr);
  }

  front_distance_fov = scan_degrees_for_attenuation * M_PI / 180.0;

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
  ROS_INFO("Scan_degrees_for_attenuation: %2.2f", scan_degrees_for_attenuation);

  nh_.param("K_wall_repulsion", K_wall_repulsion, 1.0); // speed in m/s for the repulsion after penetrating 1 m in the forbidden area
  ROS_INFO("K_wall_repulsion: %2.2f", K_wall_repulsion);

  nh_.param("min_distance_ceiling", min_distance_ceiling, 2.0); // minimum distance allowed from the ceiling
  ROS_INFO("Min_distance_ceiling: %2.2f", min_distance_ceiling);

  nh_.param("K_ceiling_repulsion", K_ceiling_repulsion, 1.0); // speed in m/s for the repulsion after penetrating 1 m in the forbidden area
  ROS_INFO("K_ceiling_repulsion: %2.2f", K_ceiling_repulsion);

  nh_.param("attenuation_distance_ceiling", attenuation_distance_ceiling, 3.0); // distance from the ceiling in meters to start attenuating the speed
  ROS_INFO("Attenuation_distance_ceiling: %2.2f", attenuation_distance_ceiling);

  nh_.param("max_height", max_height, 4.0);
  ROS_INFO("Max_height: %2.2f", max_height);

  nh_.param("attenuation_max_height", attenuation_max_height, 3.0); // height in meters to start attenuating the positive vertical speed
  ROS_INFO("Attenuation_max_height: %2.2f", attenuation_max_height);

  nh_.param("K_max_height_attraction", K_max_height_attraction, 1.0);
  ROS_INFO("K_max_height_attraction: %2.2f", K_max_height_attraction); //speed in m/s for the attraction to the ground after trespassing 1 m the maximum height allowed

  checkParameters();

  desired_vel_received = false;
  position_ctrl_vel_received = false;
  laser_scan_received = false;
  height_received = false;
  distance_ceiling_received = false;

  front_distance_fov = scan_degrees_for_attenuation * M_PI / 180.0;

  position_control_granted = false;
  //prevent all the autonomous behaviours
  nh_.setParam("position_control_granted", false);

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist_out", 1);
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_obstacles", 1);
  range_pub_ = nh_.advertise<sensor_msgs::Range>("mean_distance_front", 1);
  main_ori_pub_ = nh_.advertise<std_msgs::Float32>("orientation_front", 1);
  mean_ori_pub_ = nh_.advertise<std_msgs::Float32>("mean_orientation_front", 1);

  // Subscribers
  user_twist_subs_ = nh_.subscribe("user_twist", 1, &SafetyManager::userTwistClb, this);
  position_ctrl_twist_subs_ = nh_.subscribe("position_ctrl_twist", 1, &SafetyManager::positionCtrlTwistClb, this);
  laser_scan_subs_ = nh_.subscribe("laser_scan", 1, &SafetyManager::laserScanClb, this);
  height_subs_ = nh_.subscribe("height", 1, &SafetyManager::heightClb, this);
  ceiling_distance_subs_ = nh_.subscribe("ceiling_distance", 1, &SafetyManager::ceilingDistanceClb, this);

  // Advertising Services
  request_control_srv_ = nh_.advertiseService("request_control", &SafetyManager::requestControl, this);
  give_up_control_srv_ = nh_.advertiseService("give_up_control", &SafetyManager::giveUpControl, this);

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

  min_distance_ceiling = abs(min_distance_ceiling);
  attenuation_distance_ceiling = abs(attenuation_distance_ceiling);
  K_ceiling_repulsion = abs(K_ceiling_repulsion);

  max_height = abs(max_height);
  attenuation_max_height = abs(attenuation_max_height);
  K_max_height_attraction = abs(K_max_height_attraction);

  if(attenuation_distance_wall <= min_distance_wall){
    attenuation_distance_wall = min_distance_wall + 1;
    ROS_WARN("attenuation_distance_wall must be larger than min_distance_wall");
    ROS_WARN("attenuation_distance_wall set to %2.2f", attenuation_distance_wall);
  }

  if(attenuation_distance_ceiling <= min_distance_ceiling){
    attenuation_distance_ceiling = min_distance_ceiling + 1;
    ROS_WARN("attenuation_distance_ceiling must be larger than min_distance_ceiling");
    ROS_WARN("attenuation_distance_ceiling set to %2.2f", attenuation_distance_ceiling);
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

  if(!position_control_granted){

    position_control_granted = true;
    res.allowed = true;
    ROS_INFO("Allowing autonomous behaviour");

    //allow autonomous behaviours
    nh_.setParam("position_control_granted", true);

  }else{
    res.allowed = false;
    ROS_WARN("Autonomous behaviour was already allowed");
  }

  return true;

}

bool SafetyManager::giveUpControl(srv_mav_behaviours::GiveUpControl::Request &req, srv_mav_behaviours::GiveUpControl::Response &res){

  if(position_control_granted){

    position_control_granted = false;
    res.ok = true;
    ROS_INFO("No autonomous behaviours in course");

    //stop all the autonomous behaviours
    nh_.setParam("position_control_granted", false);

  }else{
    res.ok = false;
    ROS_WARN("Autonomous behaviours were not allowed");
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

    if(std::isfinite(range)){

      mean_range += range;
      good_ranges ++;

    }

  }

  if(std::isfinite(laser_scan_msg->ranges[laser_half_filter])){ // good_ranges is at least 1

    laser_scan.ranges[laser_half_filter] = mean_range / good_ranges;

  }

  for (int i = laser_half_filter+1; i < laser_num_ranges - laser_half_filter; i++){ 

    float oldRange = laser_scan_msg->ranges[i-laser_half_filter-1];
    float newRange = laser_scan_msg->ranges[i+laser_half_filter];
    float range = laser_scan_msg->ranges[i];

    if(std::isfinite(oldRange)){
      mean_range -= oldRange;
      good_ranges --;
    }

    if(std::isfinite(newRange)){
      mean_range += newRange;
      good_ranges ++;
    }
    if(std::isfinite(range)) laser_scan.ranges[i] = mean_range / good_ranges; // good_ranges is at least 1

  }

  laser_pub_.publish(laser_scan);

  laser_scan_received = true;

}

void SafetyManager::heightClb(const srv_mav_msgs::MAVVerticalState::ConstPtr& height_msg){

  height = height_msg->z;
  height_received = true;

}

void SafetyManager::ceilingDistanceClb(const sensor_msgs::Range::ConstPtr& ceiling_distance_msg){

  distance_ceiling = ceiling_distance_msg->range;
  distance_ceiling_received = true;

}

void SafetyManager::timerClb(const ros::TimerEvent& event){

  if(!(desired_vel_received && laser_scan_received && height_received && distance_ceiling_received)) return;

  // get the desired command

  double desired_vx, desired_vy, desired_vz, desired_vyaw;

  desired_vx = user_desired_vel.linear.x;
  desired_vy = user_desired_vel.linear.y;
  desired_vz = user_desired_vel.linear.z;
  desired_vyaw = user_desired_vel.angular.z;

  if(position_control_granted){

    if((desired_vx == 0.0) && (desired_vy == 0.0)){

      desired_vx = position_ctrl_vel.linear.x;
      desired_vy = position_ctrl_vel.linear.y;
      desired_vz = position_ctrl_vel.linear.z;

    }else{ // the autonomous behaviour can be stopped sending commands in vX or vY

      position_control_granted = false;
      ROS_WARN("Stopping autonomous behaviour");

      //stop all the autonomous behaviours
      nh_.setParam("position_control_granted", false);

    }

  }

  // attenuate the desired command in XY with the proximity of obstacles
  attenuateXYProximity(desired_vx, desired_vy);

  // attenuate the desired command in Z with the proximity to the ceiling
  attenuateZProximity(desired_vz);

  // attenuate the desired command in Z with the proximity to the maximum height allowed
  attenuateZMaxHeight(desired_vz);

  // compute the repulsions from the surrounding obstacles
  double vx_rep, vy_rep;
  computeXYRepulsion(vx_rep, vy_rep);

  // compute the repulsions from the ceiling
  double vz_rep;
  computeZRepulsion(vz_rep);

  // compute the attraction to the ground when trespassing the maximum height allowed
  double vz_att;
  computeZAttraction(vz_att);

  // compute final velocity command
  double final_vx, final_vy, final_vz, final_vyaw;
  final_vx = desired_vx + vx_rep;
  final_vy = desired_vy + vy_rep;
  final_vz = desired_vz + vz_rep + vz_att;
  final_vyaw = desired_vyaw;

  // limit with the maximum speed allowed

  double final_vxy = sqrt(final_vx*final_vx + final_vy*final_vy);
  if (final_vxy > max_speed_xy){
    
    final_vx = (final_vx / final_vxy) * max_speed_xy;
    final_vy = (final_vy / final_vxy) * max_speed_xy;

  }
  if (final_vz > max_speed_z) final_vz = max_speed_z;
  else if (final_vz < -max_speed_z) final_vz = -max_speed_z;

  // publish final velocity command

  geometry_msgs::TwistPtr final_twist(new geometry_msgs::Twist);
  final_twist->linear.x = final_vx;
  final_twist->linear.y = final_vy;
  final_twist->linear.z = final_vz;
  final_twist->angular.z = final_vyaw;

  twist_pub_.publish(final_twist);

  // compute and publish the mean distance to the front wall

  sensor_msgs::RangePtr range_front(new sensor_msgs::Range);
  range_front->header = laser_scan.header;
  range_front->radiation_type = 1;
  range_front->min_range = laser_scan.range_min;
  range_front->max_range = laser_scan.range_max;
  range_front->field_of_view = front_distance_fov;
  range_front->range = getMeanDistanceFront();
  range_pub_.publish(range_front);

  // compute and publish the main orientation regarding the front wall

  std_msgs::Float32Ptr main_ori_front(new std_msgs::Float32);
  main_ori_front->data = getOrientationFrontMain();
  main_ori_pub_.publish(main_ori_front);

  // compute and publish the mean orientation regarding the front wall

  std_msgs::Float32Ptr mean_ori_front(new std_msgs::Float32);
  mean_ori_front->data = getOrientationFrontMean();
  mean_ori_pub_.publish(mean_ori_front);

}

void SafetyManager::attenuateXYProximity(double & x_vel, double & y_vel){

  double angle = atan2(y_vel, x_vel);

  int index = round((angle - laser_angle_min) / laser_angle_incr);

  int initial_index = std::max(0, index - half_scans_attenuation);
  int final_index = std::min(laser_num_ranges-1, index + half_scans_attenuation);

  float min_range = 100.0;

  for (int i = initial_index; i <= final_index; i++){

    float range = laser_scan.ranges[i];

    if (std::isfinite(range)){

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

    if (std::isfinite(range)){

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

void SafetyManager::attenuateZProximity(double & z_vel){

  double attenuation = 1.0;

  if(z_vel > 0.0){ // only  if we want to go higher

    // Ds = distance_ceiling - min_distance_ceiling              --> Distance to the stop fence
    // Dsp = std::max(0.0, Ds)                                   --> Ds must be positive. If Ds is negative the attenuation is complete
    // Da = attenuation_distance_ceiling - min_distance_ceiling  --> Distance from the attenuation fence to the stop fence
    // P = Dsp / Da                                              --> Situation between fences given as a proportion. It P > 1 there is no attenuation
    // attenuation = std::min(1.0, P)

    attenuation = std::min(1.0, std::max(0.0, distance_ceiling - min_distance_ceiling) / (attenuation_distance_ceiling - min_distance_ceiling));

  }

  //attenuation is in [0.0, 1.0]

  z_vel = z_vel * attenuation;

}

void SafetyManager::computeZRepulsion(double & vz_rep){

  vz_rep = 0.0;

  // Dt = max(0.0, min_distance_ceiling-height)  --> Indicates how much we have trespassed the stop fence. A negative value means no attraction.
  // A = K_ceiling_repulsion * Dt                --> Repulsion speed 
  // attraction = std::min(max_speed_z, A)       --> Limit repulsion with the maximum speed allowed

  // negative speed to make the MAV descend
  vz_rep = -std::min(max_speed_z, K_ceiling_repulsion * std::max(0.0, min_distance_ceiling-distance_ceiling));

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
  // attraction = std::min(max_speed_z, A) --> Limit attraction with the maximum speed allowed

  // negative speed to make the MAV descend
  vz_att = -std::min(max_speed_z, K_max_height_attraction * std::max(0.0, height-max_height));

}

float SafetyManager::getMeanDistanceFront(){

  int central_range = laser_num_ranges / 2;

  int initial_range = central_range - half_scans_attenuation;
  int final_range = central_range + half_scans_attenuation;

  float mean_range = 0.0;
  int num_elem = 0;

  for (int i = initial_range; i <= final_range; i++){

    float range = laser_scan.ranges[i];

    if (std::isfinite(range)){

      mean_range += range;
      num_elem ++;

    }

  }

  mean_range = mean_range / num_elem;

  return mean_range;

}

double SafetyManager::getOrientationFrontMean(){

  int central_range = laser_num_ranges / 2;

  int initial_range = central_range - half_scans_attenuation;
  int final_range = central_range + half_scans_attenuation;

  int iter = 0;
  double sum_X = 0.0;
  double sum_Y = 0.0;
  double sum_XX = 0.0;
  double sum_YY = 0.0;
  double sum_XY = 0.0;
  int num_elem = 0;

  for (int i = initial_range; i <= final_range; i++){

    double range = laser_scan.ranges[i];

    if (std::isfinite(range)){

      double alpha = -(half_scans_attenuation-iter) * laser_angle_incr;
      double x = range*cos(alpha);
      double y = range*sin(alpha);

      sum_X += x;
      sum_Y += y;
      sum_XX += x*x;
      sum_YY += y*y;
      sum_XY += x*y;

      num_elem ++;

    }

    iter ++;

  }

  double sXX = num_elem * (sum_XX/num_elem - (sum_X/num_elem)*(sum_X/num_elem));
  double sYY = num_elem * (sum_YY/num_elem - (sum_Y/num_elem)*(sum_Y/num_elem));
  double sXY = num_elem * (sum_XY/num_elem - (sum_X/num_elem)*(sum_Y/num_elem));

  bool isHorizontal = sXY == 0 && sXX < sYY;
  bool isVertical = sXY == 0 && sXX > sYY;
  bool isIndeterminate = sXY == 0 && sXX == sYY;
  double mav_ori;

  if (isHorizontal){

    mav_ori = 0.0;
    ROS_INFO("wall horizontal");

  }else if (isVertical){

    ROS_INFO("wall vertical");
    mav_ori = M_PI_2;

  }else if (isIndeterminate){

    ROS_INFO("wall indeterminate");
    mav_ori = M_PI_2; // not real

  }else{

    double lambda = ((sXX+sYY)-sqrt((sXX+sYY)*(sXX+sYY)-4*(sXX*sYY-sXY*sXY))) / 2.0;
    double slope = -sXY/(sXX-lambda);
    mav_ori = -atan2(1.0, slope);
    if (mav_ori < -M_PI_2) mav_ori += M_PI;

  }

  return mav_ori * 180.0 / M_PI;

}

double SafetyManager::getOrientationFrontMain(){

  int offset_range = 1;

  int central_range = laser_num_ranges / 2;

  int initial_range = central_range - half_scans_attenuation;
  int final_range = central_range + half_scans_attenuation;

  int iter = 0;

  std::vector<int> angles_v (181,0);// from -90 to 90 degrees including 0

  for (int i = initial_range; i <= final_range - offset_range; i+=offset_range){

    double range1 = laser_scan.ranges[i];
    double range2 = laser_scan.ranges[i+offset_range];

    if (std::isfinite(range1) && (std::isfinite(range2))){

      double alpha = -(half_scans_attenuation-iter) * laser_angle_incr;
      double x1 = range1*cos(alpha);
      double y1 = range1*sin(alpha);

      alpha = -(half_scans_attenuation-(iter+offset_range)) * laser_angle_incr;
      double x2 = range2*cos(alpha);
      double y2 = range2*sin(alpha);

      double beta = atan2((x2-x1),(y2-y1)) * 180.0 / M_PI;

      angles_v[90+int(round(beta))]++;

    }

    iter+=offset_range;

  }

  std::vector<int>::iterator main_angle_votes = max_element(angles_v.begin(),angles_v.end());
  int main_angle_pose = distance(angles_v.begin(), main_angle_votes);

  int total_samples = angles_v[main_angle_pose] + angles_v[main_angle_pose-1] + angles_v[main_angle_pose+1];
  double main_angle = -90.0 + double(main_angle_pose * angles_v[main_angle_pose] + (main_angle_pose-1) * angles_v[main_angle_pose-1] + (main_angle_pose+1) * angles_v[main_angle_pose+1]) / total_samples;

  return main_angle;

}

}  // namespace srv_mav_behaviours
