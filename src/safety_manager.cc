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

  max_speed = config.max_speed;

  min_distance_wall = config.min_distance_wall; // minimum distance allowed from walls
  attenuation_distance_wall = config.attenuation_distance_wall; // distance from the wall in meters to start attenuating the speed
  degrees_for_attenuation = config.degrees_for_attenuation; // angular sector of the point cloud considered when computing the attenuation (in degrees)
  K_wall_repulsion = config.K_wall_repulsion; // speed in m/s for the repulsion after penetrating 1 m in the forbidden area

  min_distance_ceiling = config.min_distance_ceiling; // minimum distance allowed from the ceiling
  attenuation_distance_ceiling = config.attenuation_distance_ceiling; // distance from the ceiling in meters to start attenuating the speed
  K_ceiling_repulsion = config.K_ceiling_repulsion; // speed in m/s for the repulsion after penetrating 1 m in the forbidden area

  max_height = config.max_height;
  attenuation_max_height = config.attenuation_max_height; // height in meters to start attenuating the vertical speed
  K_max_height_attraction = config. K_max_height_attraction; //speed in m/s for the attraction to the ground after trespassing 1 m the maximum height allowed

  checkParameters();

}

void SafetyManager::configure(){
  
  double frequency;
  nh_.param("frequency", frequency, 50.0);
  ROS_INFO("Frequency: %2.2f", frequency);

  nh_.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO("Base Frame: %s", base_frame.c_str());

  nh_.param("robot_radius", robot_radius, 0.5);
  ROS_INFO("Robot radius: %2.2f", robot_radius);

  nh_.param("remove_ground_distance", remove_ground_distance, 2.0);
  ROS_INFO("Remove ground distance: %2.2f", remove_ground_distance);

  nh_.param("max_speed", max_speed, 1.0);
  ROS_INFO("Max_speed: %2.2f", max_speed);

  nh_.param("min_distance_wall", min_distance_wall, 1.2); // minimum distance allowed from walls
  ROS_INFO("Min_distance_wall: %2.2f", min_distance_wall);

  nh_.param("attenuation_distance_wall", attenuation_distance_wall, 2.0); // distance from the wall in meters to start attenuating the speed
  ROS_INFO("Attenuation_distance_wall: %2.2f", attenuation_distance_wall);

  nh_.param("degrees_for_attenuation", degrees_for_attenuation, 40.0); // angular sector of the point_cloud considered when computing the attenuation (in degrees)
  ROS_INFO("Degrees_for_attenuation: %2.2f", degrees_for_attenuation);

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

  flight_status = 0;

  desired_vel_received = false;
  position_ctrl_vel_received = false;
  yaw_ctrl_vel_received = false;
  point_cloud_received = false;
  height_received = false;
  //distance_ceiling_received = false;
  distance_ceiling_received = true; // Teraranger removed. No distance to the ceiling available!
  distance_ground_received = false;
  imu_received = false;

  distance_ceiling = 10.0; // Teraranger removed. No distance to the ceiling available!

  position_control_granted = false;
  yaw_control_granted = false;
  //prevent all the autonomous behaviours
  nh_.setParam("position_control_granted", false);
  nh_.setParam("yaw_control_granted", false);

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist_out", 1);
  mean_dist_front_pub_ = nh_.advertise<sensor_msgs::Range>("mean_distance_front", 1);
  wall_ori_front_pub_ = nh_.advertise<std_msgs::Float32>("orientation_front", 1);
  wall_tilt_front_pub_ = nh_.advertise<std_msgs::Float32>("wall_tilt_front", 1);
  min_dist_front_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_front", 1);
  min_dist_left_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_left", 1);
  min_dist_right_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_right", 1);
  min_dist_back_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_back", 1);
  min_dist_front_left_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_front_left", 1);
  min_dist_front_right_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_front_right", 1);
  min_dist_back_left_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_back_left", 1);
  min_dist_back_right_pub_ = nh_.advertise<sensor_msgs::Range>("min_distance_back_right", 1);
  point_cloud_pub_ = nh_.advertise<PointCloud>("obstacles", 1);

  // Subscribers
  user_twist_subs_ = nh_.subscribe("user_twist", 1, &SafetyManager::userTwistClb, this);
  position_ctrl_twist_subs_ = nh_.subscribe("position_ctrl_twist", 1, &SafetyManager::positionCtrlTwistClb, this);
  yaw_ctrl_twist_subs_ = nh_.subscribe("yaw_ctrl_twist", 1, &SafetyManager::yawCtrlTwistClb, this);
  point_cloud_subs_ = nh_.subscribe<PointCloud>("point_cloud", 1, &SafetyManager::pointCloudClb, this);
  imu_subs_ = nh_.subscribe("imu", 1, &SafetyManager::imuClb, this);
  height_subs_ = nh_.subscribe("height", 1, &SafetyManager::heightClb, this);
  ceiling_distance_subs_ = nh_.subscribe("ceiling_distance", 1, &SafetyManager::ceilingDistanceClb, this);
  ground_distance_subs_ = nh_.subscribe("ground_distance", 1, &SafetyManager::groundDistanceClb, this);
  flight_status_subs_ = nh_.subscribe("flight_status", 1, &SafetyManager::flightStatusClb, this);

  // Advertising Services
  request_position_control_srv_ = nh_.advertiseService("request_position_control", &SafetyManager::requestPositionControl, this);
  give_up_position_control_srv_ = nh_.advertiseService("give_up_position_control", &SafetyManager::giveUpPositionControl, this);
  request_yaw_control_srv_ = nh_.advertiseService("request_yaw_control", &SafetyManager::requestYawControl, this);
  give_up_yaw_control_srv_ = nh_.advertiseService("give_up_yaw_control", &SafetyManager::giveUpYawControl, this);

  //Service clients

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &SafetyManager::timerClb, this);

}

void SafetyManager::checkParameters(){

  robot_radius = abs(robot_radius);

  max_speed = abs(max_speed);

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

bool SafetyManager::requestPositionControl(srv_mav_behaviours::RequestPositionControl::Request &req, srv_mav_behaviours::RequestPositionControl::Response &res){

  if(flight_status == 3){ // the vehicle is flying
    if(!position_control_granted){

      position_control_granted = true;
      res.allowed = true;
      ROS_INFO("Allowing autonomous behaviour");

      //allow autonomous behaviours
      nh_.setParam("position_control_granted", true);

    }else{

      res.allowed = true;
      ROS_DEBUG("Autonomous behaviour was already allowed");

    }

  }else{ // the vehicle is not flying

    position_control_granted = false;
    res.allowed = false;
    ROS_WARN("Autonomous behaviours are not allowed on ground");

  }

  return true;

}

bool SafetyManager::giveUpPositionControl(srv_mav_behaviours::GiveUpPositionControl::Request &req, srv_mav_behaviours::GiveUpPositionControl::Response &res){

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

bool SafetyManager::requestYawControl(srv_mav_behaviours::RequestYawControl::Request &req, srv_mav_behaviours::RequestYawControl::Response &res){

  if(flight_status == 3){ // the vehicle is flying
    if(!yaw_control_granted){

      yaw_control_granted = true;
      res.allowed = true;
      ROS_INFO("Allowing autonomous yaw control");

      //allow autonomous yaw control
      nh_.setParam("yaw_control_granted", true);

    }else{

      res.allowed = true;
      ROS_DEBUG("Autonomous yaw control was already allowed");

    }

  }else{ // the vehicle is not flying

    yaw_control_granted = false;
    res.allowed = false;
    ROS_WARN("Autonomous yaw control is not allowed on ground");

  }

  return true;

}

bool SafetyManager::giveUpYawControl(srv_mav_behaviours::GiveUpYawControl::Request &req, srv_mav_behaviours::GiveUpYawControl::Response &res){

  if(yaw_control_granted){

    yaw_control_granted = false;
    res.ok = true;
    ROS_INFO("No autonomous yaw control in course");

    //stop the autonomous yaw control
    nh_.setParam("yaw_control_granted", false);

  }else{
    res.ok = false;
    ROS_WARN("Autonomous yaw control was not allowed");
  }

  return true;

}

void SafetyManager::flightStatusClb(const std_msgs::UInt8::ConstPtr& flight_status_msg){

  flight_status = flight_status_msg->data;

}

void SafetyManager::positionCtrlTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  position_ctrl_vel = *twist_msg;
  position_ctrl_vel_received = true;

}

void SafetyManager::yawCtrlTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  yaw_ctrl_vel = *twist_msg;
  yaw_ctrl_vel_received = true;

}

void SafetyManager::userTwistClb(const geometry_msgs::Twist::ConstPtr& twist_msg){

  user_desired_vel = *twist_msg;
  desired_vel_received = true;

}

void SafetyManager::pointCloudClb(const PointCloud::ConstPtr& point_cloud_msg){

  if(!(imu_received && distance_ground_received)) return;

  //remove the robot parts from the point cloud
  pcl::ConditionOr<Point>::Ptr range_cond(new pcl::ConditionOr<Point>);//Instantiate condition pointer
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("x", pcl::ComparisonOps::GT, robot_radius)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("x", pcl::ComparisonOps::LT, -robot_radius)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("y", pcl::ComparisonOps::GT, robot_radius)));
  range_cond->addComparison(pcl::FieldComparison<Point>::ConstPtr(new pcl::FieldComparison<Point>("y", pcl::ComparisonOps::LT, -robot_radius)));
  //build the filter
  pcl::ConditionalRemoval<Point> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(point_cloud_msg);
  condrem.setKeepOrganized(false);//Preserving the original point cloud structure means that the number of points is not reduced, and nan is used instead
  //apply filter
  condrem.filter(point_cloud);

  //listen for OS1_sensor to base_frame transform and transform the point_cloud
  tf::StampedTransform bslk2OS1;
  try{
    tf_lis_.lookupTransform("/"+base_frame, point_cloud_msg->header.frame_id, ros::Time(0), bslk2OS1);
  }catch (tf::TransformException &ex) {
    ROS_WARN("Could NOT get TF between baselink and %s: %s", point_cloud_msg->header.frame_id.c_str(), ex.what());
    return;
  }

  // tf::Matrix3x3 m_os1;
  // m_os1 = bslk2OS1.getBasis();
  // tf::Vector3 v_os1;
  // v_os1 = bslk2OS1.getOrigin();

  // Eigen::Matrix3d m_os1_eig;
  // tf::matrixTFToEigen(m_os1, m_os1_eig);
  // Eigen::Vector3d v_os1_eig;
  // tf::vectorTFToEigen(v_os1,v_os1_eig);

  // Eigen::Matrix4d m_os1_eig4 = Eigen::Matrix4d::Identity();
  // m_os1_eig4.block(0,0,3,3) = m_os1_eig;
  // m_os1_eig4.block(0,4,1,1) = v_os1_eig;

  // pcl::transformPointCloud (point_cloud, point_cloud, m_os1_eig4);

  tf::Transform tf_aux (bslk2OS1);
  Eigen::Isometry3d iso_eig;
  tf::transformTFToEigen(tf_aux, iso_eig);
  pcl::transformPointCloud (point_cloud, point_cloud,iso_eig.matrix());

  point_cloud.header.frame_id = base_frame;

  //compensate robot roll and pitch
    
  tf::Quaternion quat;
  tf::quaternionMsgToTF(imu.orientation, quat);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  tf::Matrix3x3 m_hori;
  m_hori.setRPY(roll, pitch, 0.0);

  Eigen::Matrix3d m_eig;
  tf::matrixTFToEigen(m_hori, m_eig);

  Eigen::Matrix4d m_eig4 = Eigen::Matrix4d::Identity();
  m_eig4.block(0,0,3,3) = m_eig;
  pcl::transformPointCloud (point_cloud, point_cloud, m_eig4);

  point_cloud.header.frame_id = base_frame+"_hori";

  //publish TF from base_frame to base_frame_hori
  tf::Transform bslk2bslk_hori;
  tf::Quaternion q;
  q.setRPY(-roll, -pitch, 0.0);
  bslk2bslk_hori.setRotation(q);
  bslk2bslk_hori.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf_br_.sendTransform(tf::StampedTransform(bslk2bslk_hori, ros::Time::now(), base_frame, base_frame+"_hori"));
 

  
  pcl::PassThrough<Point> pass;
  pass.setFilterFieldName("z");
  if(distance_ground < remove_ground_distance){ //flying close to the ground
    pass.setFilterLimits(0.0, attenuation_distance_wall); //eliminate all the ground points from the input pointcloud
  }else{
    pass.setFilterLimits(-attenuation_distance_wall, attenuation_distance_wall); //consider only obstacles close to the XY plane of the robot
  }
  pass.setInputCloud(point_cloud.makeShared());
  pass.filter(point_cloud);

  point_cloud_received = true;

  point_cloud_pub_.publish(point_cloud);

}

void SafetyManager::heightClb(const srv_mav_msgs::MAVVerticalState::ConstPtr& height_msg){

  height = height_msg->z;
  height_received = true;

}

void SafetyManager::ceilingDistanceClb(const sensor_msgs::Range::ConstPtr& ceiling_distance_msg){

  if(std::isfinite(ceiling_distance_msg->range)){
    distance_ceiling = ceiling_distance_msg->range;
  }else{
    distance_ceiling = 5.0; //we received a nan or inf
  }	 

  distance_ceiling_received = true;
}

void SafetyManager::groundDistanceClb(const sensor_msgs::Range::ConstPtr& ground_distance_msg){

  if(std::isfinite(ground_distance_msg->range)){
    distance_ground = ground_distance_msg->range;
  }else{
    distance_ground = 5.0; //we received a nan or inf
  }	 

  distance_ground_received = true;
}

void SafetyManager::imuClb(const sensor_msgs::Imu::ConstPtr& imu_msg){
 
  imu = *imu_msg; 
  imu_received = true;

}

void SafetyManager::timerClb(const ros::TimerEvent& event){

  if(!(desired_vel_received && point_cloud_received  && imu_received && height_received && distance_ceiling_received && distance_ground_received)) return;

  // get the desired command

  double desired_vx, desired_vy, desired_vz, desired_vyaw;

  desired_vx = user_desired_vel.linear.x;
  desired_vy = user_desired_vel.linear.y;
  desired_vz = user_desired_vel.linear.z;
  desired_vyaw = user_desired_vel.angular.z;

  if(!position_control_granted){

    position_ctrl_vel_received = false;

  }
  if(!yaw_control_granted){

    yaw_ctrl_vel_received = false;

  }
  
  if(position_control_granted && position_ctrl_vel_received){

    if((desired_vx == 0.0) && (desired_vy == 0.0) && (flight_status == 3)){

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

  if(yaw_control_granted && yaw_ctrl_vel_received){

    if((desired_vyaw == 0.0) && (flight_status == 3)){

      desired_vyaw = yaw_ctrl_vel.angular.z;

    }else{ // the autonomous yaw control can be stopped sending commands in vYaw

      yaw_control_granted = false;
      ROS_WARN("Stopping autonomous yaw control");

      //stop the autonomous yaw control
      nh_.setParam("yaw_control_granted", false);

    }
  }

  // attenuate the desired command in XY with the proximity of obstacles
  attenuateXYZProximity(desired_vx, desired_vy, desired_vz);

  // attenuate the desired command in Z with the proximity to the ceiling
  attenuateZProximity(desired_vz);

  // attenuate the desired command in Z with the proximity to the maximum height allowed
  attenuateZMaxHeight(desired_vz);

  // compute the repulsions from the surrounding obstacles
  double vx_rep, vy_rep, vz_rep;
  computeXYZRepulsion(vx_rep, vy_rep, vz_rep);

  // compute the repulsions from the ceiling
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

  double final_vel = sqrt(final_vx*final_vx + final_vy*final_vy + final_vz*final_vz);
  if (final_vel > max_speed){
    
    final_vx = (final_vx / final_vel) * max_speed;
    final_vy = (final_vy / final_vel) * max_speed;
    final_vz = (final_vz / final_vel) * max_speed;

  }

  // publish final velocity command

  geometry_msgs::TwistPtr final_twist(new geometry_msgs::Twist);
  final_twist->linear.x = final_vx;
  final_twist->linear.y = final_vy;
  final_twist->linear.z = final_vz;
  final_twist->angular.z = final_vyaw;

  twist_pub_.publish(final_twist);

  // compute and publish the mean distance and orientation regarding the front wall

  double tilt, skew, dis;
  getPlaneParams(tilt, skew, dis);

  std_msgs::Float32Ptr wall_ori_front(new std_msgs::Float32);
  wall_ori_front->data = skew;
  wall_ori_front_pub_.publish(wall_ori_front);

  std_msgs::Float32Ptr wall_tilt_front(new std_msgs::Float32);
  wall_tilt_front->data = tilt;
  wall_tilt_front_pub_.publish(wall_tilt_front);

  sensor_msgs::RangePtr range_mean_front(new sensor_msgs::Range);
  range_mean_front->header.stamp = ros::Time::now();
  range_mean_front->range = dis;
  mean_dist_front_pub_.publish(range_mean_front);

  // compute and publish the minimum distance to the front

  sensor_msgs::RangePtr range_min_front(new sensor_msgs::Range);
  range_min_front = range_mean_front;
  range_min_front->range = getMinDistance(3);
  min_dist_front_pub_.publish(range_min_front);

  // compute and publish the minimum distance to the left

  sensor_msgs::RangePtr range_min_left(new sensor_msgs::Range);
  range_min_left = range_mean_front;
  range_min_left->range = getMinDistance(5);
  min_dist_left_pub_.publish(range_min_left);

  // compute and publish the minimum distance to the right

  sensor_msgs::RangePtr range_min_right(new sensor_msgs::Range);
  range_min_right = range_mean_front;
  range_min_right->range = getMinDistance(1);
  min_dist_right_pub_.publish(range_min_right);

  // compute and publish the minimum distance to the back

  sensor_msgs::RangePtr range_min_back(new sensor_msgs::Range);
  range_min_back = range_mean_front;
  range_min_back->range = getMinDistance(7);
  min_dist_back_pub_.publish(range_min_back);

  // compute and publish the minimum distance to the front_left

  sensor_msgs::RangePtr range_min_front_left(new sensor_msgs::Range);
  range_min_front_left = range_mean_front;
  range_min_front_left->range = getMinDistance(4);
  min_dist_front_left_pub_.publish(range_min_front_left);

  // compute and publish the minimum distance to the front_right

  sensor_msgs::RangePtr range_min_front_right(new sensor_msgs::Range);
  range_min_front_right = range_mean_front;
  range_min_front_right->range = getMinDistance(2);
  min_dist_front_right_pub_.publish(range_min_front_right);

  // compute and publish the minimum distance to the back_left

  sensor_msgs::RangePtr range_min_back_left(new sensor_msgs::Range);
  range_min_back_left = range_mean_front;
  range_min_back_left->range = getMinDistance(6);
  min_dist_back_left_pub_.publish(range_min_back_left);

  // compute and publish the minimum distance to the back_right

  sensor_msgs::RangePtr range_min_back_right(new sensor_msgs::Range);
  range_min_back_right = range_mean_front;
  range_min_back_right->range = getMinDistance(0);
  min_dist_back_right_pub_.publish(range_min_back_right);

}

void SafetyManager::attenuateXYZProximity(double & x_vel, double & y_vel, double & z_vel){

  PointCloud::Ptr point_cloud_ptr = point_cloud.makeShared();

  // point_cloud_pub_.publish(point_cloud_ptr);
  // return;

  pcl::FrustumCulling<Point> fc;
  fc.setInputCloud (point_cloud_ptr);
  fc.setVerticalFOV (degrees_for_attenuation);
  fc.setHorizontalFOV (degrees_for_attenuation);
  fc.setNearPlaneDistance (robot_radius);
  fc.setFarPlaneDistance (attenuation_distance_wall);

  Eigen::Quaternionf rot;
  rot.setFromTwoVectors(Eigen::Vector3f(1.0,0.0,0.0), Eigen::Vector3f(x_vel, y_vel, z_vel));

  Eigen::Matrix3f mat3 = rot.toRotationMatrix();
  Eigen::Matrix4f pose_orig = Eigen::Matrix4f::Identity();
  pose_orig.block(0,0,3,3) = mat3;

  Eigen::Matrix4f cam2robot;
  cam2robot << 1, 0, 0, 0,
               0, 0, 1, 0,
               0,-1, 0, 0,
               0, 0, 0, 1;

  Eigen::Matrix4f camera_pose = pose_orig * cam2robot; // rotate 90 degrees around the robot x axis

  // std::cout << "Pose orig:\n" << pose_orig << std::endl;
  // std::cout << "cam to robot:\n" << cam2robot << std::endl;
  // std::cout << "Camera pose:\n" << camera_pose << std::endl;
  // std::cout << "------------------------" << std::endl;

  fc.setCameraPose(camera_pose);
  
  PointCloud target;
  fc.filter (target);

  // Publish the filtered point cloud
  // point_cloud_pub_.publish(target);

  //compute the closest point in the frustrum pointcloud
  float min_range = 10000.0;
  float dist_i;
  for(size_t i=0; i<target.points.size(); i++){

    dist_i = target.points[i].x*target.points[i].x + target.points[i].y*target.points[i].y + target.points[i].z*target.points[i].z;
    if(dist_i < min_range) min_range = dist_i;

  }
  min_range = sqrt(min_range);

  // Ds = min_range - min_distance_wall                  --> Distance to the stop fence
  // Dsp = std::max(0.0, Ds)                             --> Ds must be positive. If Ds is negative the attenuation is complete
  // Da = attenuation_distance_wall - min_distance_wall  --> Distance from the attenuation fence to the stop fence
  // P = Dsp / Da                                        --> Situation between fences given as a proportion. It P > 1 there is no attenuation
  // attenuation = std::min(1.0, P)

  double attenuation = std::min(1.0, std::max(0.0, min_range - min_distance_wall) / (attenuation_distance_wall - min_distance_wall));

  // ROS_INFO("min_range: %f", min_range);
  // ROS_INFO("Attenuation: %f", attenuation);

  //attenuation is in [0.0, 1.0]

  x_vel = x_vel * attenuation;
  y_vel = y_vel * attenuation;
  z_vel = z_vel * attenuation;

  // ROS_INFO("vel: %f, %f, %f", x_vel, y_vel, z_vel);

}

void SafetyManager::computeXYZRepulsion(double & vx_rep, double & vy_rep, double & vz_rep){

  vx_rep = vy_rep = vz_rep = 0.0;

  int num_rep = 0;
  float dist_i_P2;
  float min_distance_wall_P2 = min_distance_wall*min_distance_wall;

  for(size_t i=0; i<point_cloud.points.size(); i++){

    dist_i_P2 = point_cloud.points[i].x*point_cloud.points[i].x + point_cloud.points[i].y*point_cloud.points[i].y + point_cloud.points[i].z*point_cloud.points[i].z;

    if(dist_i_P2 < min_distance_wall_P2){

        // Dt = min_distance_wall-range          --> Indicates how much we have trespassed the stop fence. It is always positive.
        // R = K_wall_repulsion * Dt             --> Repulsion speed 
        // repulsion = std::min(max_speed_xy, R) --> Limit repulsion with the maximum speed allowed

        // double repulsion = std::min(max_speed_xy, K_wall_repulsion * (min_distance_wall-sqrt(dist_i)));
        double dist_i = sqrt(dist_i_P2);
        double repulsion = K_wall_repulsion * (min_distance_wall-dist_i);

        // repulsion is in [0.0, max_speed_xy]        <-- false
        // repulsion is in [0.0, K_wall_repulsion*Dt] <-- true

        vx_rep -= repulsion * point_cloud.points[i].x/dist_i;
        vy_rep -= repulsion * point_cloud.points[i].y/dist_i;
        vz_rep -= repulsion * point_cloud.points[i].z/dist_i;
        num_rep ++;
    }

  }

  // compute the mean repulsion vector
  if(num_rep > 0){

    vx_rep = vx_rep / num_rep;
    vy_rep = vy_rep / num_rep;
    vz_rep = vz_rep / num_rep;

  }

  // finally saturate using the max_speed parameter
  double rep_vel = sqrt(vx_rep*vx_rep + vy_rep*vy_rep + vz_rep*vz_rep);
  if (rep_vel > max_speed){
    
    vx_rep = (vx_rep / rep_vel) * max_speed;
    vy_rep = (vy_rep / rep_vel) * max_speed;
    vz_rep = (vz_rep / rep_vel) * max_speed;

  }

  // ROS_INFO("vel: %f, %f, %f", vx_rep, vy_rep, vz_rep);

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

  // Dt = max(0.0, min_distance_ceiling-distance_ceiling)  --> Indicates how much we have trespassed the stop fence. A negative value means no repulsion.
  // R = K_ceiling_repulsion * Dt                --> Repulsion speed 
  // repulsion = std::min(max_speed_z, R)        --> Limit repulsion with the maximum speed allowed

  // negative speed to make the MAV descend
  vz_rep = -std::min(max_speed, K_ceiling_repulsion * std::max(0.0, min_distance_ceiling-distance_ceiling));

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
  vz_att = -std::min(max_speed, K_max_height_attraction * std::max(0.0, height-max_height));

}

float SafetyManager::getMinDistance(int direction){

  /*direction:
      0--> back_right
      1--> right
      2--> front_right
      3--> front
      4--> front_left
      5--> left
      6--> back_left
      7--> back
  */

  float x_comp, y_comp, z_comp;

  if (direction == 0){
    x_comp = -1.0;
    y_comp = -1.0;
    z_comp = 0.0;
  }else if (direction == 1){
    x_comp = 0.0;
    y_comp = -1.0;
    z_comp = 0.0;
  }else if (direction == 2){
    x_comp = 1.0;
    y_comp = -1.0;
    z_comp = 0.0;
  }else if (direction == 3){
    x_comp = 1.0;
    y_comp = 0.0;
    z_comp = 0.0;
  }else if (direction == 4){
    x_comp = 1.0;
    y_comp = 1.0;
    z_comp = 0.0;
  }else if (direction == 5){
    x_comp = 0.0;
    y_comp = 1.0;
    z_comp = 0.0;
  }else if (direction == 6){
    x_comp = -1.0;
    y_comp = 1.0;
    z_comp = 0.0;
  }else if (direction == 7){
    x_comp = -1.0;
    y_comp = 0.0;
    z_comp = 0.0;
  }

  PointCloud::Ptr point_cloud_ptr = point_cloud.makeShared();

  pcl::FrustumCulling<Point> fc;
  fc.setInputCloud (point_cloud_ptr);
  fc.setVerticalFOV (degrees_for_attenuation);
  fc.setHorizontalFOV (degrees_for_attenuation);
  fc.setNearPlaneDistance (robot_radius);
  fc.setFarPlaneDistance (100.0);

  Eigen::Quaternionf rot;
  rot.setFromTwoVectors(Eigen::Vector3f(1.0,0.0,0.0), Eigen::Vector3f(x_comp, y_comp, z_comp));

  Eigen::Matrix3f mat3 = rot.toRotationMatrix();
  Eigen::Matrix4f pose_orig = Eigen::Matrix4f::Identity();
  pose_orig.block(0,0,3,3) = mat3;

  Eigen::Matrix4f cam2robot;
  cam2robot << 1, 0, 0, 0,
               0, 0, 1, 0,
               0,-1, 0, 0,
               0, 0, 0, 1;

  Eigen::Matrix4f camera_pose = pose_orig * cam2robot; // rotate 90 degrees around the robot x axis

  // std::cout << "Pose orig:\n" << pose_orig << std::endl;
  // std::cout << "cam to robot:\n" << cam2robot << std::endl;
  // std::cout << "Camera pose:\n" << camera_pose << std::endl;
  // std::cout << "------------------------" << std::endl;

  fc.setCameraPose(camera_pose);
  
  PointCloud target;
  fc.filter (target);

  // Publish the filtered point cloud
  // point_cloud_pub_.publish(target);

  //compute the closest point in the frustrum pointcloud
  float min_range = 10000.0;
  float dist_i;
  for(size_t i=0; i<target.points.size(); i++){

    dist_i = target.points[i].x*target.points[i].x + target.points[i].y*target.points[i].y + target.points[i].z*target.points[i].z;
    if(dist_i < min_range) min_range = dist_i;

  }
  min_range = sqrt(min_range);

  return min_range;

}

void SafetyManager::getPlaneParams(double & tilt, double & skew, double & distance){

  PointCloud::Ptr point_cloud_ptr = point_cloud.makeShared();

  tilt = skew = 0.0;
  distance = 100.0;

  float x_comp, y_comp, z_comp;

  x_comp = 1.0;
  y_comp = 0.0;
  z_comp = 0.0;

  pcl::FrustumCulling<Point> fc;
  fc.setInputCloud (point_cloud_ptr);
  //fc.setVerticalFOV (degrees_for_attenuation);
  //fc.setHorizontalFOV (degrees_for_attenuation);
  fc.setVerticalFOV (60);
  fc.setHorizontalFOV (60);
  fc.setNearPlaneDistance (robot_radius);
  fc.setFarPlaneDistance (100.0);

  Eigen::Quaternionf rot;
  rot.setFromTwoVectors(Eigen::Vector3f(1.0,0.0,0.0), Eigen::Vector3f(x_comp, y_comp, z_comp));

  Eigen::Matrix3f mat3 = rot.toRotationMatrix();
  Eigen::Matrix4f pose_orig = Eigen::Matrix4f::Identity();
  pose_orig.block(0,0,3,3) = mat3;

  Eigen::Matrix4f cam2robot;
  cam2robot << 1, 0, 0, 0,
               0, 0, 1, 0,
               0,-1, 0, 0,
               0, 0, 0, 1;

  Eigen::Matrix4f camera_pose = pose_orig * cam2robot; // rotate 90 degrees around the robot x axis

  fc.setCameraPose(camera_pose);
  
  PointCloud target;
  fc.filter (target);

  if(target.width > 5){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<Point> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (target.makeShared());

    try
    {
      seg.segment (*inliers, *coefficients);
    }
    
    catch (std::exception& e)
    {
      ROS_WARN("RANSAC error: %s", e.what());
    }
   
    // if we have just a few inliers and these are not almost all the possible cadidates
    if ((inliers->indices.size() < 10) && (inliers->indices.size() < (target.width * 0.8))){
      //ROS_WARN ("Could not estimate a good planar model for the given dataset.\n");
      return;
    }

    // ROS_INFO("RANSAC inliers: %d/%d possible", (int)(inliers->indices.size()), target.width); 

    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    if(D > 0.0){
      A*=-1;
      B*=-1;
      C*=-1;
      D*=-1;
    }

    // std::cerr << "Model coefficients: " << A << " " 
    //                                     << B << " "
    //                                     << C << " " 
    //                                     << D << std::endl;

    // distance =  -D/A;
    distance = -D / (A*x_comp + B*y_comp + C*z_comp); //<-- general case

    //compute plane orientation (tilt and skew)
    Eigen::Quaternionf orien;
    orien.setFromTwoVectors(Eigen::Vector3f(x_comp,y_comp,z_comp), Eigen::Vector3f(A, B, C));

    tf::Quaternion quat(orien.x(), orien.y(), orien.z(), orien.w());
    tf::Matrix3x3 m(quat);
    double plane_roll, plane_pitch, plane_yaw;
    m.getRPY(plane_roll, plane_pitch, plane_yaw);
    plane_roll *= 180.0/M_PI;
    plane_pitch *= 180.0/M_PI;
    plane_yaw *= 180.0/M_PI;

    tilt = plane_pitch;
    skew = plane_yaw;

    // ROS_INFO("Distance to plane: %2.2f", distance);
    // ROS_INFO("Plane roll, pitch, yaw: %2.2f, %2.2f, %2.2f", plane_roll, plane_pitch, plane_yaw);
  }

  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (const auto& idx: inliers->indices)
  //   std::cerr << idx << "    " << target.points[idx].x << " "
  //                              << target.points[idx].y << " "
  //                              << target.points[idx].z << std::endl;

}

}  // namespace srv_mav_behaviours
