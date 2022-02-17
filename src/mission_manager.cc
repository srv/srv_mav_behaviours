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

#include "srv_mav_behaviours/mission_manager.h"

namespace srv_mav_behaviours{

MissionManager::MissionManager(const ros::NodeHandle& nh) :
  nh_(nh)
{

  reconfigure_server_.setCallback(boost::bind(&MissionManager::dynReconfig, this, _1, _2));
  configure();
}

MissionManager::~MissionManager(){
}

void MissionManager::dynReconfig(srv_mav_behaviours::mission_managerConfig &config, uint32_t level){

  min_height = config.min_height;
  max_height = config.max_height;

  WP_error = config.WP_error;

  home_z = config.home_z;

  sweep_min_lateral_dist = config.sweep_min_lateral_dist;
  vinspection_min_ceiling_dist = config.vinspection_min_ceiling_dist;

  checkParameters();

}

void MissionManager::configure(){
  
  double frequency;
  nh_.param("frequency", frequency, 50.0);
  ROS_INFO("Frequency: %2.2f", frequency);

  nh_.param("min_height", min_height, 0.5);
  ROS_INFO("Min height: %2.2f", min_height); // minimum height for the autonomous behaviours

  nh_.param("max_height", max_height, 2.0);
  ROS_INFO("Max height: %2.2f", max_height); // maximum height for the autonomous behaviours

  nh_.param("WP_error", WP_error, 0.3);
  ROS_INFO("WP_error: %2.2f", WP_error);

  nh_.param("home_z", home_z, 1.5);
  ROS_INFO("Home Z: %2.2f", home_z);

  nh_.param("sweep_min_lateral_dist", sweep_min_lateral_dist, 3.0);
  ROS_INFO("Wall-to-wall sweeping min. distance: %2.2f", sweep_min_lateral_dist);

  nh_.param("vinspection_min_ceiling_dist", vinspection_min_ceiling_dist, 3.0);
  ROS_INFO("Vert. inspection up-to-ceiling min. distance: %2.2f", vinspection_min_ceiling_dist);

  checkParameters();

  pose_received = false;

  min_dist_left = min_dist_right = min_dist_up = min_dist_down = INFINITY;

  nh_.setParam("position_control_granted", false); //set to false the parameter safetyManager/position_control_granted (also done by the safetyManager)

  position_controllers_enabled = false;

  WP_x = WP_y = WP_z = 0.0;

  publish_mission_path = false;
  mission_path = nav_msgs::PathPtr(new nav_msgs::Path);

  // --------------parameters for WP-based sweeping-----------------

  performing_sweep = false;

  initial_yaw_sweep = 0.0;
  sweep_y_accumulated = 0.0;
  final_z_sweep = 0.0;
  sweep_state = 0;  //0--> go right
                    //1--> go down
                    //2--> go left
                    //3--> go down

  sweep_status = 0; //0-->No sweeping in course
                    //1-->Sweeping
                    //2-->Paused
  nh_.setParam("sweep_status", sweep_status);

  sweep_y_size = sweep_z_size = sweep_y_increment = sweep_z_increment = 0.0;

  // --------------parameters for vertical inspection-----------------

  performing_vinspection = false;

  initial_yaw_vinspection = 0.0;
  final_z_vinspection = 0.0;
  vinspection_state = 0;  //0--> go up
                          //1--> go right
                          //2--> go down

  vinspection_status = 0; //0-->No vertical inspection in course
                          //1-->Vertical inspection
                          //2-->Paused
  nh_.setParam("vinspection_status", vinspection_status);

  vinspection_y_size = vinspection_z_size = vinspection_z_increment = 0.0;

  // -------------------parameters for hovering--------------------

  nh_.setParam("hovering", false);

  // -------------------parameters for go-home---------------------

  home_x = home_y = 0.0; // home_z set through the launchfile
  nh_.setParam("going_home", false);

  // Publishers
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("way_point", 1);
  mission_path_pub_ = nh_.advertise<nav_msgs::Path>("mission_path", 1);

  // Subscribers
  pose_subs_ = nh_.subscribe("pose", 1, &MissionManager::poseClb, this);
  min_distance_left_subs_ = nh_.subscribe("min_distance_left", 1, &MissionManager::minDistanceLeftClb, this);
  min_distance_right_subs_ = nh_.subscribe("min_distance_right", 1, &MissionManager::minDistanceRightClb, this);
  min_distance_up_subs_ = nh_.subscribe("min_distance_up", 1, &MissionManager::minDistanceUpClb, this);
  min_distance_down_subs_ = nh_.subscribe("min_distance_down", 1, &MissionManager::minDistanceDownClb, this);

  // Advertising Services
  start_sweep_srv_ = nh_.advertiseService("start_sweep", &MissionManager::startSweep, this);
  stop_sweep_srv_ = nh_.advertiseService("stop_sweep", &MissionManager::stopSweep, this);
  pause_sweep_srv_ = nh_.advertiseService("pause_sweep", &MissionManager::pauseSweep, this);
  resume_sweep_srv_ = nh_.advertiseService("resume_sweep", &MissionManager::resumeSweep, this);
  start_vertical_inspection_srv_ = nh_.advertiseService("start_vertical_inspection", &MissionManager::startVerticalInspection, this);
  stop_vertical_inspection_srv_ = nh_.advertiseService("stop_vertical_inspection", &MissionManager::stopVerticalInspection, this);
  pause_vertical_inspection_srv_ = nh_.advertiseService("pause_vertical_inspection", &MissionManager::pauseVerticalInspection, this);
  resume_vertical_inspection_srv_ = nh_.advertiseService("resume_vertical_inspection", &MissionManager::resumeVerticalInspection, this);
  hover_srv_ = nh_.advertiseService("hover", &MissionManager::hover, this);
  go_home_srv_ = nh_.advertiseService("go_home", &MissionManager::goHome, this);
  set_home_srv_ = nh_.advertiseService("set_home", &MissionManager::setHome, this);
  save_point_srv_ = nh_.advertiseService("save_point", &MissionManager::savePoint, this);
  go_to_point_srv_ = nh_.advertiseService("go_to_point", &MissionManager::goToPoint, this);

  //Service clients
  request_control_client_ = nh_.serviceClient<srv_mav_behaviours::RequestControl>("request_control");
  give_up_control_client_ = nh_.serviceClient<srv_mav_behaviours::GiveUpControl>("give_up_control");
  enable_position_control_client_ = nh_.serviceClient<srv_mav_control::EnablePositionControl>("enable_position_control");

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &MissionManager::timerClb, this);
}

void MissionManager::checkParameters(){

  min_height = abs(min_height);
  max_height = abs(max_height);
  WP_error = abs(WP_error);
  home_z = abs(home_z);
  sweep_min_lateral_dist = abs(sweep_min_lateral_dist);
  vinspection_min_ceiling_dist = abs(vinspection_min_ceiling_dist);

  if(min_height < 0.5){

    min_height = 0.5;
    ROS_WARN("min_height set to %2.2f m", min_height);

  }

  if(max_height <= min_height){

    max_height = min_height + 1.0;
    ROS_WARN("max_height must be above min_height");
    ROS_WARN("max_height set to %2.2f", max_height);

  }else if(max_height < 2.0){

    max_height = 2.0;
    ROS_WARN("max_height set to %2.2f m", max_height);

  }

  if(home_z < 0.5){
    home_z = 0.5;
    ROS_WARN("home_z was too low. home_z is set to %2.2f", home_z);
  } 

  if(sweep_min_lateral_dist < 2.0){

    sweep_min_lateral_dist = 2.0;
    ROS_WARN("sweep_min_lateral_dist too low, sweep_min_lateral_dist set to %2.2f", sweep_min_lateral_dist);

  }

  if(vinspection_min_ceiling_dist < 1.5){

    vinspection_min_ceiling_dist = 1.5;
    ROS_WARN("vinspection_min_ceiling_dist too low, sweep_min_lateral_dist set to %2.2f", vinspection_min_ceiling_dist);

  }

}

void MissionManager::minDistanceLeftClb(const sensor_msgs::Range::ConstPtr& range_msg){

  min_dist_left = range_msg->range;

}

void MissionManager::minDistanceRightClb(const sensor_msgs::Range::ConstPtr& range_msg){

  min_dist_right = range_msg->range;

}

void MissionManager::minDistanceUpClb(const sensor_msgs::Range::ConstPtr& range_msg){

  min_dist_up = range_msg->range;

}

void MissionManager::minDistanceDownClb(const sensor_msgs::Range::ConstPtr& range_msg){

  min_dist_down = range_msg->range;

}

bool MissionManager::startSweep(srv_mav_behaviours::StartSweep::Request &req, srv_mav_behaviours::StartSweep::Response &res){

  if(!pose_received) return false;

  if(performing_sweep || (sweep_status == 2)){
    ROS_WARN("Sweep already in process!!");
    return false;
  }

  if(current_z < min_height){
    ROS_WARN("Flying too low to start a sweep");
    return true;
  }

  if(min_dist_down < min_height){
    ROS_WARN("Obstacle below the MAV"); 
    return true;
  }

  //load the sweeping parameters
  sweep_y_size = req.width;
  sweep_z_size = req.height;
  sweep_y_increment = req.horizontal_step;
  sweep_z_increment = req.vertical_step;
  sweep_wall_to_wall = req.wall_to_wall;

  if(((sweep_y_size <= 0.0) && !sweep_wall_to_wall) || (sweep_z_size <= 0.0)){

    ROS_WARN("Unspecified sweep dimensions");
    return false;

  }

  if(sweep_wall_to_wall){

    if(sweep_y_increment < 0.5) sweep_y_increment = 0.5;

  }else{

    if((sweep_y_increment > sweep_y_size) || (sweep_y_increment <= 0.0)) sweep_y_increment = sweep_y_size;

  }

  if((sweep_z_increment > sweep_z_size) || (sweep_z_increment == 0.0)) sweep_z_increment = sweep_z_size;

  if((min_dist_right - sweep_y_increment) < sweep_min_lateral_dist){

    ROS_WARN("Too close to the wall to start a sweep");
    return false;

  }

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // start the sweep

    //the sweep starts from the top left corner

    //compute the first displacement
    tf::Vector3 robot_incr(0.0, -sweep_y_increment, 0.0); //move to the right

     //rotate the increment to the world frame using the estimated yaw
    tf::Matrix3x3 m_rot;
    m_rot.setRPY(0, 0, current_yaw);
    tf::Vector3 world_incr = m_rot * robot_incr;

    //compute the first WP
    WP_x = current_x + world_incr.getX();
    WP_y = current_y + world_incr.getY();
    WP_z = current_z;
    initial_yaw_sweep = current_yaw;
    sweep_y_accumulated = 0.0;
    final_z_sweep = current_z - sweep_z_size;

    performing_sweep = true;  
    sweep_status = 1;
    nh_.setParam("sweep_status", sweep_status);
  
    sweep_state = 0; // going to the right
    ROS_WARN("Starting new sweep");

    sweep_reaching_end = false;

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);
    performing_vinspection = false;
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);

    if(!sweep_wall_to_wall){
      createSweepingPath();
      publish_mission_path = true;
    }
  }

  return true;
}

bool MissionManager::stopSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_sweep){

    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    ROS_WARN("Sweeping stopped");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

  }else if(sweep_status == 2){ // the last sweeping is paused

    // performing_sweep is already false
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    ROS_WARN("Sweeping stopped");

  }else{

    ROS_WARN("No sweeping in course");

  }

  return true;
}

bool MissionManager::pauseSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_sweep){

    performing_sweep = false;
    sweep_status = 2;
    nh_.setParam("sweep_status", sweep_status);
    ROS_WARN("Sweeping paused");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

    // save WP to allow resuming the sweeping
    pausedSW_WP_x = WP_x;
    pausedSW_WP_y = WP_y;
    pausedSW_WP_z = WP_z;

  }else{

    ROS_WARN("No sweeping in course");

  }

  return true;
}

bool MissionManager::resumeSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(sweep_status == 2){ // the last sweeping is paused

    // request control to the Safety Manager
    srv_mav_behaviours::RequestControl request_control;
    request_control_client_.call(request_control);

    if(request_control.response.allowed){ // resume the sweep

      //recompute the WP with the current orientation

      // restore saved WP
      WP_x = pausedSW_WP_x;
      WP_y = pausedSW_WP_y;
      WP_z = pausedSW_WP_z;

      if((sweep_state == 0) || (sweep_state == 2)){ // not going down

        double remaining_x = WP_x - current_x;
        double remaining_y = WP_y - current_y;
        double sweep_y_remaining = sqrt(remaining_x*remaining_x + remaining_y*remaining_y);
        
        if(sweep_state == 0){ // go to right

          sweep_y_remaining = -sweep_y_remaining;

        }

        tf::Vector3 robot_incr(0.0, sweep_y_remaining, 0.0); //move to the right

        //rotate the increment to the world frame using the estimated yaw
        tf::Matrix3x3 m_rot;
        m_rot.setRPY(0, 0, current_yaw);
        tf::Vector3 world_incr = m_rot * robot_incr;

        //update the next WP
        WP_x = current_x + world_incr.getX();
        WP_y = current_y + world_incr.getY();

      } 
      
      // update the inital_yaw_sweep for computing the rest of waypoints
      initial_yaw_sweep = current_yaw;
      
      performing_sweep = true;  
      sweep_status = 1;
      nh_.setParam("sweep_status", sweep_status);
    
      ROS_WARN("Resuming sweep");

      //stop all the other behaviours
      nh_.setParam("hovering", false);
      nh_.setParam("going_home", false);
      nh_.setParam("going_to_point", false);
      performing_vinspection = false;
      vinspection_status = 0;
      nh_.setParam("vinspection_status", vinspection_status);

    }

  }else{

    ROS_WARN("No sweeping in pause");

  }

  return true;
}

bool MissionManager::startVerticalInspection(srv_mav_behaviours::StartVerticalInspection::Request &req, srv_mav_behaviours::StartVerticalInspection::Response &res){

  if(!pose_received) return false;

  if(performing_vinspection || (vinspection_status == 2)){
    ROS_WARN("Vertical inspection already in process!!");
    return false;
  }

  if(current_z > max_height){
    ROS_WARN("Flying too high to start a vertical inspection");
    return true;
  }

  if(min_dist_down < min_height){
    ROS_WARN("Obstacle below the MAV"); 
    return true;
  }

  //load the sweeping parameters
  vinspection_y_size = req.width;
  vinspection_z_size = req.height;
  vinspection_z_increment = req.vertical_step;
  vinspection_to_ceiling = req.to_ceiling;

  if((vinspection_y_size <= 0.0) || ((vinspection_z_size <= 0.0) && !vinspection_to_ceiling)){

    ROS_WARN("Unspecified vertical inspection dimensions");
    return false;

  }

  if(vinspection_to_ceiling){

    if(vinspection_z_increment < 0.5) vinspection_z_increment = 0.5;

  }else{

    if((vinspection_z_increment > vinspection_z_size) || (vinspection_z_increment <= 0.0)) vinspection_z_increment = vinspection_z_size;

  }

  if((min_dist_up - vinspection_z_increment) < vinspection_min_ceiling_dist){

    ROS_WARN("Too close to the ceiling to start a vertical inspection");
    return false;

  }

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // start the vertical inspection

    //the vertical inspection starts from the bottom left corner

    //compute the first WP
    WP_x = current_x;
    WP_y = current_y;
    WP_z = current_z + vinspection_z_increment;
    initial_yaw_vinspection = current_yaw;
    vinspection_z_accumulated = 0.0;
    final_z_vinspection = current_z;

    performing_vinspection = true;  
    vinspection_status = 1;
    nh_.setParam("vinspection_status", vinspection_status);
  
    vinspection_state = 0; // going up
    ROS_WARN("Starting new vertical inspection");

    vinspection_reaching_end = false;

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);
    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
  }

  return true;
}

bool MissionManager::stopVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_vinspection){

    performing_vinspection = false;
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);
    ROS_WARN("Vertical inspection stopped");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

  }else if(vinspection_status == 2){ // the last vertical inspection is paused

    // performing_sweep is already false
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);
    ROS_WARN("Vertical inspection stopped");

  }else{

    ROS_WARN("No vertical inspection in course");

  }

  return true;
}

bool MissionManager::pauseVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_vinspection){

    performing_vinspection = false;
    vinspection_status = 2;
    nh_.setParam("vinspection_status", vinspection_status);
    ROS_WARN("Vertical inspection paused");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

    // save WP to allow resuming the vertical inspection
    pausedSW_WP_x = WP_x;
    pausedSW_WP_y = WP_y;
    pausedSW_WP_z = WP_z;

  }else{

    ROS_WARN("No vertical inspection in course");

  }

  return true;
}

bool MissionManager::resumeVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(vinspection_status == 2){ // the last vertical inspection is paused

    // request control to the Safety Manager
    srv_mav_behaviours::RequestControl request_control;
    request_control_client_.call(request_control);

    if(request_control.response.allowed){ // resume the vertical inspection

      //recompute the WP with the current orientation

      // restore saved WP
      WP_x = pausedSW_WP_x;
      WP_y = pausedSW_WP_y;
      WP_z = pausedSW_WP_z;

      if((vinspection_state == 0) || (vinspection_state == 2)){ // going up or down

        //update the next WP
        WP_x = current_x;
        WP_y = current_y;

      }else{ // going to the right

        // update the inital_yaw_vinspection for computing the next waypoint
        initial_yaw_vinspection = current_yaw;

        double remaining_x = WP_x - current_x;
        double remaining_y = WP_y - current_y;
        double vinspection_y_remaining = sqrt(remaining_x*remaining_x + remaining_y*remaining_y);

        tf::Vector3 robot_incr(0.0, -vinspection_y_remaining, 0.0); //move to the right

        //rotate the increment to the world frame using the estimated yaw
        tf::Matrix3x3 m_rot;
        m_rot.setRPY(0, 0, initial_yaw_vinspection);
        tf::Vector3 world_incr = m_rot * robot_incr;

        //update the next WP
        WP_x = current_x + world_incr.getX();
        WP_y = current_y + world_incr.getY();

      }
      
      performing_vinspection = true;  
      vinspection_status = 1;
      nh_.setParam("vinspection_status", vinspection_status);
    
      ROS_WARN("Resuming vertical inspection");

      //stop all the other behaviours
      nh_.setParam("hovering", false);
      nh_.setParam("going_home", false);
      nh_.setParam("going_to_point", false);
      performing_sweep = false;
      sweep_status = 0;
      nh_.setParam("sweep_status", sweep_status);

    }

  }else{

    ROS_WARN("No vertical inspection in pause");

  }

  return true;
}

bool MissionManager::hover(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!pose_received) return false;

  performHovering();
  return true;
}

void MissionManager::performHovering(){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // hover

    nh_.setParam("hovering", true);

    //stop all other behaviours
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);

    if(performing_sweep){//pause the sweeping in course (if any)

      sweep_status = 2;
      nh_.setParam("sweep_status", sweep_status);
      ROS_WARN("Sweeping paused");
      performing_sweep = false;

      // save WP to allow resuming the sweeping
      pausedSW_WP_x = WP_x;
      pausedSW_WP_y = WP_y;
      pausedSW_WP_z = WP_z;

    }

    if(performing_vinspection){//pause the vertical inspection in course (if any)

      vinspection_status = 2;
      nh_.setParam("vinspection_status", vinspection_status);
      ROS_WARN("Vertical inspection paused");
      performing_vinspection = false;

      // save WP to allow resuming the vertical inspection
      pausedSW_WP_x = WP_x;
      pausedSW_WP_y = WP_y;
      pausedSW_WP_z = WP_z;

    }

    ROS_WARN("Hovering at %2.2f, %2.2f, %2.2f", current_x, current_y, current_z);

    //update the WP to the current position
    WP_x = current_x;
    WP_y = current_y;
    WP_z = current_z;

  }

}

bool MissionManager::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!pose_received) return false;

  performGoHome();
  return true;
}

void MissionManager::performGoHome(){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // go home

    nh_.setParam("going_home", true);

    //stop all other behaviours
    nh_.setParam("going_to_point", false);
    nh_.setParam("hovering", false);
    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    performing_vinspection = false;
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);

    ROS_WARN("Going home: %2.2f, %2.2f, %2.2f", home_x, home_y, home_z);

    //update the WP to the current position
    WP_x = home_x;
    WP_y = home_y;
    WP_z = home_z;

  }

}

bool MissionManager::setHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!pose_received) return false;
  
  home_x = current_x;
  home_y = current_y;

  //write the home point in the parameter server for the GUI
  nh_.setParam("home_x", home_x);
  nh_.setParam("home_y", home_y);
  nh_.setParam("home_z", home_z);

  ROS_INFO("New home point: %2.2f, %2.2f, %2.2f", home_x, home_y, home_z);

  return true;
}

bool MissionManager::savePoint(srv_mav_behaviours::SavePoint::Request &req, srv_mav_behaviours::SavePoint::Response &res){

  // save current_x, current_y, current_z and the description received

  saved_positions_x.push_back(current_x);
  saved_positions_y.push_back(current_y);
  saved_positions_z.push_back(current_z);
  saved_positions_description.push_back(req.description);

  nh_.setParam("saved_positions/x", saved_positions_x);
  nh_.setParam("saved_positions/y", saved_positions_y);
  nh_.setParam("saved_positions/z", saved_positions_z);
  nh_.setParam("saved_positions/description", saved_positions_description);

  ROS_INFO("Point saved: %2.2f, %2.2f, %2.2f (%s)", current_x, current_y, current_z, req.description.c_str());

  res.ok = true;

  return true;
}

bool MissionManager::goToPoint(srv_mav_behaviours::GoToPoint::Request &req, srv_mav_behaviours::GoToPoint::Response &res){

  performGoToPoint(req.x, req.y, req.z);

  return true;
}

void MissionManager::performGoToPoint(double point_x, double point_y, double point_z){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // go to point

    nh_.setParam("going_to_point", true);

    //stop all other behaviours
    nh_.setParam("going_home", false);
    nh_.setParam("hovering", false);
    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    performing_vinspection = false;
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);

    ROS_WARN("Going to point: %2.2f, %2.2f, %2.2f", point_x, point_y, point_z);

    //update the WP to the received point
    WP_x = point_x;
    WP_y = point_y;
    WP_z = point_z;

  }

}

void MissionManager::poseClb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){

  current_x = pose_msg->pose.pose.position.x;
  current_y = pose_msg->pose.pose.position.y;
  current_z = pose_msg->pose.pose.position.z;

  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_msg->pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double curr_roll, curr_pitch;
  m.getRPY(curr_roll, curr_pitch, current_yaw);

  pose_received = true;

}

void MissionManager::timerClb(const ros::TimerEvent& event){

  if(!(pose_received)) return;

  bool position_control_granted;
  nh_.getParam("position_control_granted", position_control_granted);

  if(position_control_granted && (min_dist_down < min_height)){

    ROS_WARN("Obstacle below the MAV");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);
    nh_.getParam("position_control_granted", position_control_granted);

  }

  if (!position_control_granted){//stop all the behaviours

    if(performing_sweep){//pause the sweeping in course (if any)

      sweep_status = 2;
      nh_.setParam("sweep_status", sweep_status);
      ROS_WARN("Sweeping paused");
      performing_sweep = false;

      // save WP to allow resuming the sweeping
      pausedSW_WP_x = WP_x;
      pausedSW_WP_y = WP_y;
      pausedSW_WP_z = WP_z;

    }

    if(performing_vinspection){//pause the vertical inspection in course (if any)

      vinspection_status = 2;
      nh_.setParam("vinspection_status", vinspection_status);
      ROS_WARN("Vertical inspection paused");
      performing_vinspection = false;

      // save WP to allow resuming the vertical inspection
      pausedSW_WP_x = WP_x;
      pausedSW_WP_y = WP_y;
      pausedSW_WP_z = WP_z;

    }

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);

  }

  if(performing_sweep){

    performSweep(); //it provides the WP

  }

  if(performing_vinspection){

    performVerticalInspection(); //it provides the WP

  }

  if(position_control_granted){ // perform some mission/behaviour

    // publish the current WP
    geometry_msgs::PosePtr pose_WP(new geometry_msgs::Pose);
    pose_WP->position.x = WP_x;
    pose_WP->position.y = WP_y;
    pose_WP->position.z = WP_z;

    pose_pub_.publish(pose_WP);

    if(!position_controllers_enabled){

      //enable position control
      srv_mav_control::EnablePositionControl enable_pos_ctrl;
      enable_pos_ctrl.request.enable = 1;
      enable_position_control_client_.call(enable_pos_ctrl);

      if(enable_pos_ctrl.response.enabled){
        position_controllers_enabled = true;
      }

    }

  }else{ // do not perform any behaviour/mission

    if(position_controllers_enabled){

      //disable position_control
      srv_mav_control::EnablePositionControl enable_pos_ctrl;
      enable_pos_ctrl.request.enable = 0;
      enable_position_control_client_.call(enable_pos_ctrl);

      if(!enable_pos_ctrl.response.enabled){
        position_controllers_enabled = false;
      }
    }

  }

  if(publish_mission_path){
    if (sweep_status > 0){//sweeping in progress or paused
      publishMissionPath();
    }else{
      clearMissionPath();
      publishMissionPath(); // publish one last time an empty path
      publish_mission_path = false;
    }
  }

}

void MissionManager::performSweep(){

  double errorX = WP_x-current_x;
  double errorY = WP_y-current_y;
  double errorZ = WP_z-current_z;
  double errorXY = sqrt(errorX*errorX + errorY*errorY);
  double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

  if(errorWP < WP_error){ // the WP has been reached

    // update the sweep_state if necessary

    if((sweep_state == 0) || (sweep_state == 2)){ // going to the right or to the left

      sweep_y_accumulated += sweep_y_increment;

      if(!sweep_wall_to_wall){

        if(sweep_y_accumulated >= sweep_y_size){ // lateral movement finished

          sweep_state ++;
          sweep_state = sweep_state%4;
          sweep_y_accumulated = 0.0;

        } //else: keep going in that direction

      }else{// wall-to-wall sweeping

        // double lateral_dist = (sweep_state == 0) ? min_dist_right : min_dist_left;

        // if(lateral_dist < (sweep_min_lateral_dist + WP_error)) { // wall found

        if(sweep_reaching_end){

          sweep_state ++;
          sweep_state = sweep_state%4;
          sweep_y_accumulated = 0.0;
          sweep_reaching_end = false;

        }//else: keep going in that direction
      }

    } else{ // going down

      sweep_state ++;
      sweep_state = sweep_state%4;

    }

    // compute the next WP

    if((sweep_state == 1) || (sweep_state == 3)){ // lets go down

      double aux_WP_z = WP_z - sweep_z_increment;

      if(aux_WP_z < final_z_sweep){

        ROS_WARN("Sweep finised");

        performing_sweep = false;
        sweep_status = 0;
        nh_.setParam("sweep_status", sweep_status);

      }else if(aux_WP_z < min_height){

        ROS_WARN("Flying too low to keep on sweeping");

        performing_sweep = false;
        sweep_status = 0;
        nh_.setParam("sweep_status", sweep_status);

      }else{

        WP_z = aux_WP_z; // we update WPz only if it is reachable 

      }

    }else{ //lets go to the right or to the left

      double robot_incr_y = sweep_y_increment;

      if(!sweep_wall_to_wall){

        if((sweep_y_accumulated + sweep_y_increment) > sweep_y_size){

          robot_incr_y = sweep_y_size - sweep_y_accumulated; // the remaining displacement (lower than sweep_y_increment)

        }

      }else{// wall-to-wall sweeping

        double wall_dist = (sweep_state == 0) ? min_dist_right : min_dist_left;

        if((wall_dist - errorXY - sweep_y_increment) < sweep_min_lateral_dist){

          robot_incr_y = wall_dist - errorXY - sweep_min_lateral_dist; // the remaining displacement (lower than sweep_y_increment)
          sweep_reaching_end = true;

        }
      }

      if(sweep_state == 0){ // lets go to the right

        robot_incr_y = -robot_incr_y;

      }

      //rotate the increment to the world frame using the initial estimated yaw
      tf::Vector3 robot_incr(0.0, robot_incr_y, 0.0);
      tf::Matrix3x3 m_rot;
      m_rot.setRPY(0, 0, initial_yaw_sweep);
      tf::Vector3 world_incr = m_rot * robot_incr;

      WP_x = WP_x + world_incr.getX();
      WP_y = WP_y + world_incr.getY();

    }

  } 

  if(!performing_sweep){ // the sweeping has finished now

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

  }

}

void MissionManager::performVerticalInspection(){

  double errorX = WP_x-current_x;
  double errorY = WP_y-current_y;
  double errorZ = WP_z-current_z;

  double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

  if(errorWP < WP_error){ // the WP has been reached

    // update the sweep_state if necessary

    if((vinspection_state == 0) || (vinspection_state == 2)){ // going up or down

      vinspection_z_accumulated += vinspection_z_increment;

      if(!vinspection_to_ceiling){

        if (vinspection_z_accumulated >= vinspection_z_size){ // vertical movement finished

          vinspection_state ++;
          vinspection_state = vinspection_state%3;
          vinspection_z_accumulated = 0.0;

          if (vinspection_state == 0){ // the state says to go up again
            
            ROS_WARN("Vertical inspection finised");

            performing_vinspection = false;
            vinspection_status = 0;
            nh_.setParam("vinspection_status", vinspection_status);

          }

        } //else: keep going in that direction

      }else{// vertical inspection up-to-ceiling

        // if(((vinspection_state == 0) && (min_dist_up < (vinspection_min_ceiling_dist + WP_error))) || 
        //     ((vinspection_state == 2) && (current_z <= final_z_vinspection))){

        if(vinspection_reaching_end){

          vinspection_state ++;
          vinspection_state = vinspection_state%3;
          vinspection_z_accumulated = 0.0;

          if (vinspection_state == 0){ // the state says to go up again
            
            ROS_WARN("Vertical inspection finised");

            performing_vinspection = false;
            vinspection_status = 0;
            nh_.setParam("vinspection_status", vinspection_status);

          }

          vinspection_reaching_end = false;

        } //else: keep going in that direction

      }

    }else{ // going to the right

      vinspection_state = 2; //lets go down

    }

    // compute the next WP

    if(vinspection_state == 1){ // lets go to the right

      //rotate the displacement to the world frame using the initial estimated yaw
      tf::Vector3 robot_incr(0.0, -vinspection_y_size, 0.0);
      tf::Matrix3x3 m_rot;
      m_rot.setRPY(0, 0, initial_yaw_vinspection);
      tf::Vector3 world_incr = m_rot * robot_incr;

      WP_x = WP_x + world_incr.getX();
      WP_y = WP_y + world_incr.getY();

    }else if(((vinspection_state == 0) && performing_vinspection) || (vinspection_state == 2)){ // going up (and not restarting after finishing) or going down
      
      double robot_incr_z = vinspection_z_increment;

      if(!vinspection_to_ceiling){

        if((vinspection_z_accumulated + vinspection_z_increment) > vinspection_z_size){
          
          robot_incr_z = vinspection_z_size - vinspection_z_accumulated; // the remaining displacement (lower than vinspection_z_increment)
        }

        if(vinspection_state == 2){ // lets go down

          robot_incr_z = -robot_incr_z;

        }

        WP_z = WP_z + robot_incr_z;

      }else{// vert. inspection up-to-ceiling

        if (vinspection_state == 0){ // going up

          if((min_dist_up - errorZ - sweep_z_increment) < vinspection_min_ceiling_dist){

            robot_incr_z = min_dist_up - errorZ - vinspection_min_ceiling_dist; // the remaining displacement (lower than sweep_y_increment)
            vinspection_reaching_end = true;

          }

          WP_z = WP_z + robot_incr_z;

        }else{ // going down 

          WP_z = WP_z - robot_incr_z;

          if((current_z - errorZ - vinspection_z_increment) < final_z_vinspection){
            
            WP_z = final_z_vinspection;
            vinspection_reaching_end = true;
            
          }
        }

      }

    }

  } 

  if(!performing_vinspection){ // the vertical inspection has finished now

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);

  }

}

void MissionManager::publishMissionPath(){

  mission_path_pub_.publish(mission_path);

}

void MissionManager::clearMissionPath(){

  mission_path = nav_msgs::PathPtr(new nav_msgs::Path);

}

void MissionManager::createSweepingPath(){

  clearMissionPath();

  geometry_msgs::PoseStamped point1;
  geometry_msgs::PoseStamped point2;

  //add current position as initial point1
  point1.pose.position.x = current_x;
  point1.pose.position.y = current_y;
  point1.pose.position.z = current_z;
  mission_path->poses.push_back(point1);

  tf::Vector3 robot_incr(0.0, sweep_y_size, 0.0);
  tf::Matrix3x3 m_rot;
  m_rot.setRPY(0, 0, initial_yaw_sweep);
  tf::Vector3 world_incr = m_rot * robot_incr;
  double other_x = current_x + world_incr.getX();
  double other_y = current_y + world_incr.getY();

  double last_z = current_z;
  double next_z;
  double final_z = current_z - sweep_z_size;
  bool right = true;

  while(true){
    
    if(right){

      point1.pose.position.x = other_x;
      point1.pose.position.y = other_y;

    }else{ // go back to the left

      point1.pose.position.x = current_x;
      point1.pose.position.y = current_y;

    }

    point1.pose.position.z = last_z;
    right = !right;

    mission_path->poses.push_back(point1);

    next_z = last_z - sweep_z_increment; 

    if(next_z < final_z) break; // beyond sweeping vertical limit 
    else if(next_z < min_height) break; // too close to the ground
    else{

      point2.pose.position.x = point1.pose.position.x;
      point2.pose.position.y = point1.pose.position.y;
      point2.pose.position.z = next_z;
      last_z = next_z;

      mission_path->poses.push_back(point2);

    }
  }
}

}  // namespace srv_mav_behaviours
