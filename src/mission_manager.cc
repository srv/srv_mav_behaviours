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

  WP_tolerance = config.WP_tolerance;

  home_z = config.home_z;

  sweep_min_lateral_dist = config.sweep_min_lateral_dist;
  vinspection_min_ceiling_dist = config.vinspection_min_ceiling_dist;

  follow_trajectory = config.follow_trajectory;
  follow_trajectory_delta = config.follow_trajectory_delta; // maximum distance to VTP
  follow_trajectory_lambda = config.follow_trajectory_lambda; // maximum distance to the path

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

  nh_.param("WP_tolerance", WP_tolerance, 0.3);
  ROS_INFO("WP_tolerance: %2.2f", WP_tolerance);

  nh_.param("home_z", home_z, 1.5);
  ROS_INFO("Home Z: %2.2f", home_z);

  nh_.param("sweep_min_lateral_dist", sweep_min_lateral_dist, 3.0);
  ROS_INFO("Wall-to-wall sweeping min. distance: %2.2f", sweep_min_lateral_dist);

  nh_.param("vinspection_min_ceiling_dist", vinspection_min_ceiling_dist, 3.0);
  ROS_INFO("Vert. inspection up-to-ceiling min. distance: %2.2f", vinspection_min_ceiling_dist);

  nh_.param("follow_trajectory", follow_trajectory, false);
  if (follow_trajectory) ROS_INFO("Configured to follow trajectories");
  else ROS_INFO("Configured to NOT follow trajectories (only go to WPs)");

  nh_.param("follow_trajectory_delta", follow_trajectory_delta, 0.3);
  ROS_INFO("Trajectory following delta: %2.2f", follow_trajectory_delta);

  nh_.param("follow_trajectory_lambda", follow_trajectory_lambda, 0.3);
  ROS_INFO("Trajectory following lambda: %2.2f", follow_trajectory_lambda);

  checkParameters();

  odom_received = false;

  min_dist_left = min_dist_right = min_dist_up = min_dist_down = INFINITY;

  nh_.setParam("position_control_granted", false); //set to false the parameter safetyManager/position_control_granted (also done by the safetyManager)
  nh_.setParam("yaw_control_granted", false); //set to false the parameter safetyManager/yaw_control_granted (also done by the safetyManager)

  position_controllers_enabled = false;
  yaw_controller_enabled = false;

  WP_x = WP_y = WP_z = 0.0;
  WP_yaw = 0.0;

  publish_mission_path = false;
  clearMissionPath();
  newWPPath();

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

    // --------------parameters for circular inspection-----------------

  performing_cinspection = false;

  initial_yaw_cinspection = 0.0;
  cinspection_state = 0;  //0--> starting (first X degrees)
                          //1--> rest of the circumference
                          //2--> finishing
  center_x_cinspection = center_y_cinspection = 0.0;

  cinspection_status = 0; //0-->No circular inspection in course
                          //1-->Circular inspection
                          //2-->Paused
  nh_.setParam("cinspection_status", cinspection_status);

  cinspection_radius = cinspection_arc_increment = 0.0;

  // -------------------parameters for hovering--------------------

  nh_.setParam("hovering", false);

  // -------------------parameters for go-home---------------------

  home_x = home_y = 0.0; // home_z set through the launchfile
  nh_.setParam("going_home", false);

  // -------------------parameters for keeping orientation--------------------

  nh_.setParam("keeping_orientation", false);

  // Publishers
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("way_point", 1);
  mission_path_pub_ = nh_.advertise<nav_msgs::Path>("mission_path", 1);
  WP_path_pub_ = nh_.advertise<nav_msgs::Path>("wp_path", 1);

  // Subscribers
  odom_subs_ = nh_.subscribe("odom", 1, &MissionManager::odomClb, this);
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
  start_circular_inspection_srv_ = nh_.advertiseService("start_circular_inspection", &MissionManager::startCircularInspection, this);
  stop_circular_inspection_srv_ = nh_.advertiseService("stop_circular_inspection", &MissionManager::stopCircularInspection, this);
  pause_circular_inspection_srv_ = nh_.advertiseService("pause_circular_inspection", &MissionManager::pauseCircularInspection, this);
  resume_circular_inspection_srv_ = nh_.advertiseService("resume_circular_inspection", &MissionManager::resumeCircularInspection, this);
  hover_srv_ = nh_.advertiseService("hover", &MissionManager::hover, this);
  go_home_srv_ = nh_.advertiseService("go_home", &MissionManager::goHome, this);
  set_home_srv_ = nh_.advertiseService("set_home", &MissionManager::setHome, this);
  save_point_srv_ = nh_.advertiseService("save_point", &MissionManager::savePoint, this);
  go_to_point_srv_ = nh_.advertiseService("go_to_point", &MissionManager::goToPoint, this);
  keep_orientation_srv_ = nh_.advertiseService("keep_orientation", &MissionManager::keepOrientation, this);

  //Service clients
  request_position_control_client_ = nh_.serviceClient<srv_mav_behaviours::RequestPositionControl>("request_position_control");
  give_up_position_control_client_ = nh_.serviceClient<srv_mav_behaviours::GiveUpPositionControl>("give_up_position_control");
  enable_position_control_client_ = nh_.serviceClient<srv_mav_control::EnablePositionControl>("enable_position_control");
  request_yaw_control_client_ = nh_.serviceClient<srv_mav_behaviours::RequestYawControl>("request_yaw_control");
  give_up_yaw_control_client_ = nh_.serviceClient<srv_mav_behaviours::GiveUpYawControl>("give_up_yaw_control");
  enable_yaw_control_client_ = nh_.serviceClient<srv_mav_control::EnableYawControl>("enable_yaw_control");

  // Timers
  timer_ = nh_.createTimer(ros::Duration(1.0 / frequency), &MissionManager::timerClb, this);
}

void MissionManager::checkParameters(){

  min_height = abs(min_height);
  max_height = abs(max_height);
  WP_tolerance = abs(WP_tolerance);
  home_z = abs(home_z);
  sweep_min_lateral_dist = abs(sweep_min_lateral_dist);
  vinspection_min_ceiling_dist = abs(vinspection_min_ceiling_dist);
  follow_trajectory_delta = abs(follow_trajectory_delta);
  follow_trajectory_lambda = abs(follow_trajectory_lambda);

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
    ROS_WARN("vinspection_min_ceiling_dist too low, vinspection_min_ceiling_dist set to %2.2f", vinspection_min_ceiling_dist);

  }

  if(WP_tolerance < 0.1){

    WP_tolerance = 0.1;
    ROS_WARN("WP_tolerance too low, WP_tolerance set to %2.2f", WP_tolerance);

  }

  if(follow_trajectory_delta < 0.1){

    follow_trajectory_delta = 0.1;
    ROS_WARN("follow_trajectory_delta too low, follow_trajectory_delta set to %2.2f", follow_trajectory_delta);

  }

  if(follow_trajectory_lambda < 0.1){

    follow_trajectory_lambda = 0.1;
    ROS_WARN("follow_trajectory_lambda too low, follow_trajectory_lambda set to %2.2f", follow_trajectory_lambda);

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

  if(!odom_received) return false;

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
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // start the sweep

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
  
    ROS_WARN("Starting new sweeping (W: %2.2f m, H: %2.2f m, H_step: %2.2f m, incr: %2.2f m)", (sweep_wall_to_wall)?(std::numeric_limits<double>::infinity()):sweep_y_size, sweep_z_size, sweep_z_increment, sweep_y_increment);

    sweep_reaching_end = false;

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);
    performing_vinspection = false;
    vinspection_status = 0;
    nh_.setParam("vinspection_status", vinspection_status);
    performing_cinspection = false;
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);

    if(!sweep_wall_to_wall){
      createSweepingPath();
      publish_mission_path = true;
    }

    newWPPath();
    addWP2WPPath(WP_x, WP_y, WP_z);
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
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

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
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

    // save WP to allow resuming the sweeping
    pausedMission_WP_x = WP_x;
    pausedMission_WP_y = WP_y;
    pausedMission_WP_z = WP_z;

    pausedMission_initial_yaw = initial_yaw_sweep;

    removeLastWPPath();
    addWP2WPPath(current_x, current_y, current_z);

  }else{

    ROS_WARN("No sweeping in course");

  }

  return true;
}

bool MissionManager::resumeSweep(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(sweep_status == 2){ // the last sweeping is paused

    // request control to the Safety Manager
    srv_mav_behaviours::RequestPositionControl request_position_control;
    request_position_control_client_.call(request_position_control);

    if(request_position_control.response.allowed){ // resume the sweep

      //recompute the WP with the current orientation

      // restore saved WP
      WP_x = pausedMission_WP_x;
      WP_y = pausedMission_WP_y;
      WP_z = pausedMission_WP_z;

      if((sweep_state == 0) || (sweep_state == 2)){ // not going down

        // double remaining_x = WP_x - current_x;
        // double remaining_y = WP_y - current_y;
        // double sweep_y_remaining = sqrt(remaining_x*remaining_x + remaining_y*remaining_y);

        //rotate the WP and the current location -initial_yaw_sweep degrees
        //then the difference in Y axis is the sweep_y_remaining
        tf::Vector3 WP_vector(WP_x, WP_y, WP_z);
        tf::Vector3 current_vector(current_x, current_y, current_z);
        tf::Matrix3x3 yaw_SW_mat;
        yaw_SW_mat.setRPY(0, 0, -initial_yaw_sweep);
        tf::Vector3 WP_vector_rot = yaw_SW_mat * WP_vector;
        tf::Vector3 current_vector_rot = yaw_SW_mat * current_vector;
        double sweep_y_remaining = abs(WP_vector_rot.getY() - current_vector_rot.getY());
        
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
    
      ROS_WARN("Resuming sweeping (W: %2.2f m, H: %2.2f m, H_step: %2.2f m, incr: %2.2f m)", (sweep_wall_to_wall)?(std::numeric_limits<double>::infinity()):sweep_y_size, sweep_z_size, sweep_z_increment, sweep_y_increment);

      //stop all the other behaviours
      nh_.setParam("hovering", false);
      nh_.setParam("going_home", false);
      nh_.setParam("going_to_point", false);
      performing_vinspection = false;
      vinspection_status = 0;
      nh_.setParam("vinspection_status", vinspection_status);
      performing_cinspection = false;
      cinspection_status = 0;
      nh_.setParam("cinspection_status", cinspection_status);

      if(!sweep_wall_to_wall){
        recomputeSweepingPath();
      }

      addWP2WPPath(current_x, current_y, current_z);
      addWP2WPPath(WP_x, WP_y, WP_z);

    }

  }else{

    ROS_WARN("No sweeping in pause");

  }

  return true;
}

bool MissionManager::startVerticalInspection(srv_mav_behaviours::StartVerticalInspection::Request &req, srv_mav_behaviours::StartVerticalInspection::Response &res){

  if(!odom_received) return false;

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

  //load the vertical inspection parameters
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
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // start the vertical inspection

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
    ROS_WARN("Starting new vertical inspection (H: %2.2f m, W: %2.2f m, incr: %2.2f m)", (vinspection_to_ceiling)?(std::numeric_limits<double>::infinity()):vinspection_z_size, vinspection_y_size, vinspection_z_increment);

    vinspection_reaching_end = false;

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);
    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    performing_cinspection = false;
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);

    if(!vinspection_to_ceiling){
      createVerticalInspectionPath();
      publish_mission_path = true;
    }

    newWPPath();
    addWP2WPPath(WP_x, WP_y, WP_z);
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
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

  }else if(vinspection_status == 2){ // the last vertical inspection is paused

    // performing_vinspection is already false
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
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

    // save WP to allow resuming the vertical inspection
    pausedMission_WP_x = WP_x;
    pausedMission_WP_y = WP_y;
    pausedMission_WP_z = WP_z;

    pausedMission_initial_yaw = initial_yaw_vinspection;

    removeLastWPPath();
    addWP2WPPath(current_x, current_y, current_z);

  }else{

    ROS_WARN("No vertical inspection in course");

  }

  return true;
}

bool MissionManager::resumeVerticalInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(vinspection_status == 2){ // the last vertical inspection is paused

    // request control to the Safety Manager
    srv_mav_behaviours::RequestPositionControl request_position_control;
    request_position_control_client_.call(request_position_control);

    if(request_position_control.response.allowed){ // resume the vertical inspection

      //recompute the WP with the current orientation

      // restore saved WP
      WP_x = pausedMission_WP_x;
      WP_y = pausedMission_WP_y;
      WP_z = pausedMission_WP_z;

      if((vinspection_state == 0) || (vinspection_state == 2)){ // going up or down

        //update the next WP
        WP_x = current_x;
        WP_y = current_y;

      }else{ // going to the right

        // double remaining_x = WP_x - current_x;
        // double remaining_y = WP_y - current_y;
        // double vinspection_y_remaining = sqrt(remaining_x*remaining_x + remaining_y*remaining_y);

        //rotate the WP and the current location -initial_yaw_vinspection degrees
        //then the difference in Y axis is the vinspection_y_remaining
        tf::Vector3 WP_vector(WP_x, WP_y, WP_z);
        tf::Vector3 current_vector(current_x, current_y, current_z);
        tf::Matrix3x3 yaw_VI_mat;
        yaw_VI_mat.setRPY(0, 0, -initial_yaw_vinspection);
        tf::Vector3 WP_vector_rot = yaw_VI_mat * WP_vector;
        tf::Vector3 current_vector_rot = yaw_VI_mat * current_vector;
        double vinspection_y_remaining = abs(WP_vector_rot.getY() - current_vector_rot.getY());
        
        tf::Vector3 robot_incr(0.0, -vinspection_y_remaining, 0.0); //move to the right

      // update the inital_yaw_vinspection for computing the next waypoint
        initial_yaw_vinspection = current_yaw;

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
    
      ROS_WARN("Resuming vertical inspection (H: %2.2f m, W: %2.2f m, incr: %2.2f m)", (vinspection_to_ceiling)?(std::numeric_limits<double>::infinity()):vinspection_z_size, vinspection_y_size, vinspection_z_increment);

      //stop all the other behaviours
      nh_.setParam("hovering", false);
      nh_.setParam("going_home", false);
      nh_.setParam("going_to_point", false);
      performing_sweep = false;
      sweep_status = 0;
      nh_.setParam("sweep_status", sweep_status);
      performing_cinspection = false;
      cinspection_status = 0;
      nh_.setParam("cinspection_status", cinspection_status);

      if(!vinspection_to_ceiling){
        recomputeVerticalInspectionPath();
      }

      addWP2WPPath(current_x, current_y, current_z);
      addWP2WPPath(WP_x, WP_y, WP_z);

    }

  }else{

    ROS_WARN("No vertical inspection in pause");

  }

  return true;
}

bool MissionManager::startCircularInspection(srv_mav_behaviours::StartCircularInspection::Request &req, srv_mav_behaviours::StartCircularInspection::Response &res){

  if(!odom_received) return false;

  if(performing_cinspection || (cinspection_status == 2)){
    ROS_WARN("Circular inspection already in process!!");
    return false;
  }

  if(min_dist_down < min_height){
    ROS_WARN("Obstacle below the MAV"); 
    return true;
  }

  //load the circular inspection parameters
  cinspection_radius = req.radius;
  cinspection_arc_increment = req.arc_step;

  if(cinspection_radius <= 0.0 ){

    ROS_WARN("Unspecified circular inspection dimensions");
    return false;

  }

  if (cinspection_radius < 2.0){
    ROS_WARN("Circular insp. radius set to 2 m");
    cinspection_radius = 2.0;
  }

  if((cinspection_arc_increment > cinspection_radius) || (cinspection_arc_increment <= 0.0)){

    ROS_WARN("Circular insp. arc increment set to 1 m");
    cinspection_arc_increment = 1.0;
  }

  // request position control to the Safety Manager
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // request yaw control to the Safety Manager

    srv_mav_behaviours::RequestYawControl request_yaw_control;
    request_yaw_control_client_.call(request_yaw_control);

    if(request_yaw_control.response.allowed){ // start the circular inspection

      //the circular inspection starts and finishes at the same location

      //compute the first displacement
      double theta = cinspection_arc_increment/cinspection_radius; // L = theta * radius ; then theta = L/r
      double alpha = (M_PI-theta)/2.0;
      double beta = M_PI_2 - alpha;
      double d = sqrt(2*cinspection_radius*cinspection_radius * (1-cos(theta)));

      tf::Vector3 robot_incr(d*sin(beta), -d*cos(beta), 0.0); //move following a circumference

     //rotate the increment to the world frame using the estimated yaw
      tf::Matrix3x3 m_rot;
      m_rot.setRPY(0, 0, current_yaw);
      tf::Vector3 world_incr = m_rot * robot_incr;

      //compute the first WP
      WP_x = current_x + world_incr.getX();
      WP_y = current_y + world_incr.getY();
      WP_z = current_z;
      WP_yaw = current_yaw; // we will look always to the center of the circumference 
      //WP_yaw = current_yaw + theta; // we want to get the orientation of the first WP

      tf::Vector3 center_incr_robot(cinspection_radius, 0.0, 0.0); //center of the circumference
      tf::Vector3 center_incr_world = m_rot * center_incr_robot;
      center_x_cinspection = current_x + center_incr_world.getX();
      center_y_cinspection = current_y + center_incr_world.getY();
      initial_yaw_cinspection = current_yaw; // used to detect the end of the circular inspection

      performing_cinspection = true;  
      cinspection_status = 1;
      nh_.setParam("cinspection_status", cinspection_status);

      cinspection_state = 0; // going up
      ROS_WARN("Starting new circular inspection (R: %2.2f m, incr: %2.2f m)", cinspection_radius, cinspection_arc_increment);

      //stop all the other behaviours
      nh_.setParam("hovering", false);
      nh_.setParam("going_home", false);
      nh_.setParam("going_to_point", false);
      nh_.setParam("keeping_orientation", false);
      performing_sweep = false;
      sweep_status = 0;
      nh_.setParam("sweep_status", sweep_status);
      performing_vinspection = false;
      vinspection_status = 0;
      nh_.setParam("vinspection_status", vinspection_status);

      createCircularInspectionPath();
      publish_mission_path = true;

      newWPPath();
      addWP2WPPath(WP_x, WP_y, WP_z);
    }
  }

  return true;
}

bool MissionManager::stopCircularInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_cinspection){

    performing_cinspection = false;
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);
    ROS_WARN("Circular inspection stopped");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

  }else if(cinspection_status == 2){ // the last circular inspection is paused

    // performing_cinspection is already false
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);
    ROS_WARN("Circular inspection stopped");

  }else{

    ROS_WARN("No vertical inspection in course");

  }

  return true;
}

bool MissionManager::pauseCircularInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(performing_cinspection){

    performing_cinspection = false;
    cinspection_status = 2;
    nh_.setParam("cinspection_status", cinspection_status);
    ROS_WARN("Circular inspection paused");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);

    // save WP to allow resuming the circular inspection
    pausedMission_WP_x = WP_x;
    pausedMission_WP_y = WP_y;
    pausedMission_WP_z = WP_z;
    pausedMission_WP_yaw = WP_yaw;

    pausedMission_initial_yaw = initial_yaw_cinspection; // unnecessary for this behaviour

    removeLastWPPath();
    addWP2WPPath(current_x, current_y, current_z);

  }else{

    ROS_WARN("No circular inspection in course");

  }

  return true;
}

bool MissionManager::resumeCircularInspection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(cinspection_status == 2){ // the last circular inspection is paused

    // request position control to the Safety Manager
    srv_mav_behaviours::RequestPositionControl request_position_control;
    request_position_control_client_.call(request_position_control);

    if(request_position_control.response.allowed){ // request yaw control to the Safety Manager

      srv_mav_behaviours::RequestYawControl request_yaw_control;
      request_yaw_control_client_.call(request_yaw_control);

      if(request_yaw_control.response.allowed){ // resume the circular inspection

        //update the WP considering the current location and the center of the inspection for yaw
        double errorX = center_x_cinspection-current_x;
        double errorY = center_y_cinspection-current_y;
        cinspection_radius = sqrt(errorX*errorX + errorY*errorY); // distance to the center of the circumference
        if (cinspection_radius < 2.0){
          ROS_WARN("Circular insp. radius set to 2 m");
          cinspection_radius = 2.0;
        }

        //compute WP_yaw
        // double yaw_incr = acos((cos(current_yaw)*errorX + sin(current_yaw)*errorY)/ cinspection_radius);
        // double delta_aux = cos(current_yaw)*errorY - sin(current_yaw)*errorX;
        // if (delta_aux < 0.0) yaw_incr = -yaw_incr;
        // WP_yaw = current_yaw + yaw_incr;
        double error_center_x = center_x_cinspection - current_x;
        double error_center_y = center_y_cinspection - current_y;
        WP_yaw = atan2(error_center_y, error_center_x);

        //compute the displacement
        double theta = cinspection_arc_increment/cinspection_radius; // L = theta * radius ; then theta = L/r
        double alpha = (M_PI-theta)/2.0;
        double beta = M_PI_2 - alpha;
        double d = sqrt(2*cinspection_radius*cinspection_radius * (1-cos(theta)));

        tf::Vector3 robot_incr(d*sin(beta), -d*cos(beta), 0.0); //move following a circumference

        //rotate the increment to the world frame using the estimated yaw
        tf::Matrix3x3 m_rot;
        m_rot.setRPY(0, 0, WP_yaw); // use the WP_yaw instead of the current_yaw
        tf::Vector3 world_incr = m_rot * robot_incr;

        //compute the next WP
        WP_x = current_x + world_incr.getX();
        WP_y = current_y + world_incr.getY();
        WP_z = current_z;
        
        performing_cinspection = true;  
        cinspection_status = 1;
        nh_.setParam("cinspection_status", cinspection_status);
      
        ROS_WARN("Resuming circular inspection (R: %2.2f m, incr: %2.2f m)", cinspection_radius, cinspection_arc_increment);

        //stop all the other behaviours
        nh_.setParam("hovering", false);
        nh_.setParam("going_home", false);
        nh_.setParam("going_to_point", false);
        nh_.setParam("keeping_orientation", false);
        performing_sweep = false;
        sweep_status = 0;
        nh_.setParam("sweep_status", sweep_status);
        performing_vinspection = false;
        vinspection_status = 0;
        nh_.setParam("vinspection_status", vinspection_status);

        recomputeCircularInspectionPath();

        addWP2WPPath(current_x, current_y, current_z);
        addWP2WPPath(WP_x, WP_y, WP_z);

      }

    }

  }else{

    ROS_WARN("No circular inspection in pause");

  }

  return true;
}

bool MissionManager::hover(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!odom_received) return false;

  performHovering();
  return true;
}

void MissionManager::performHovering(){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // hover

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
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;

      pausedMission_initial_yaw = initial_yaw_sweep;

    }

    if(performing_vinspection){//pause the vertical inspection in course (if any)

      vinspection_status = 2;
      nh_.setParam("vinspection_status", vinspection_status);
      ROS_WARN("Vertical inspection paused");
      performing_vinspection = false;

      // save WP to allow resuming the vertical inspection
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;

      pausedMission_initial_yaw = initial_yaw_vinspection;

    }

    if(performing_cinspection){//pause the circular inspection in course (if any)

      cinspection_status = 2;
      nh_.setParam("cinspection_status", cinspection_status);
      ROS_WARN("Circular inspection paused");
      performing_cinspection = false;

      // save WP to allow resuming the circular inspection
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;
      pausedMission_WP_yaw = WP_yaw;

      pausedMission_initial_yaw = initial_yaw_cinspection; // unnecessary for this behaviour

    }

    ROS_WARN("Hovering at %2.2f, %2.2f, %2.2f", current_x, current_y, current_z);

    //update the WP to the current position
    WP_x = current_x;
    WP_y = current_y;
    WP_z = current_z;

  }

}

bool MissionManager::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!odom_received) return false;

  performGoHome();
  return true;
}

void MissionManager::performGoHome(){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // go home

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
    performing_cinspection = false;
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);

    ROS_WARN("Going home: %2.2f, %2.2f, %2.2f", home_x, home_y, home_z);

    //update the WP to the current position
    WP_x = home_x;
    WP_y = home_y;
    WP_z = home_z;

    newWPPath();
    addWP2WPPath(WP_x, WP_y, WP_z);

  }

}

bool MissionManager::setHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!odom_received) return false;
  
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

  performGoToPoint(req.x, req.y, req.z, req.looking_forward);

  return true;
}

void MissionManager::performGoToPoint(double point_x, double point_y, double point_z, bool looking_forward){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestPositionControl request_position_control;
  request_position_control_client_.call(request_position_control);

  if(request_position_control.response.allowed){ // go to point

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
    performing_cinspection = false;
    cinspection_status = 0;
    nh_.setParam("cinspection_status", cinspection_status);

    ROS_WARN("Going to point: %2.2f, %2.2f, %2.2f", point_x, point_y, point_z);

    //update the WP to the received point
    WP_x = point_x;
    WP_y = point_y;
    WP_z = point_z;

    newWPPath();
    addWP2WPPath(WP_x, WP_y, WP_z);

    if(looking_forward){

      srv_mav_behaviours::RequestYawControl request_yaw_control;
      request_yaw_control_client_.call(request_yaw_control);

      if(request_yaw_control.response.allowed){ // compute the desired yaw
        
        double error_x = WP_x - current_x;
        double error_y = WP_y - current_y;
        WP_yaw = atan2(error_y, error_x);

      }

    }

  }

}

bool MissionManager::keepOrientation(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  if(!odom_received) return false;

  if(performing_cinspection){//yaw control used by another autonomous behaviour

      ROS_WARN("Keep_orientation is not available. Another behaviour is using the yaw control");
      return false;

    }

  performKeepOrientation();
  return true;
}

void MissionManager::performKeepOrientation(){

  // request control to the Safety Manager
  srv_mav_behaviours::RequestYawControl request_yaw_control;
  request_yaw_control_client_.call(request_yaw_control);

  if(request_yaw_control.response.allowed){ // keep orientation

    nh_.setParam("keeping_orientation", true);

    ROS_WARN("Keeping orientation of %2.2f degrees", current_yaw*180.0/M_PI);

    //update the WP to the current orientation
    WP_yaw = current_yaw;

  }

}

void MissionManager::odomClb(const nav_msgs::Odometry::ConstPtr& odom_msg){

  current_x = odom_msg->pose.pose.position.x;
  current_y = odom_msg->pose.pose.position.y;
  current_z = odom_msg->pose.pose.position.z;

  world_frame = odom_msg->header.frame_id;

  tf::Quaternion q;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double curr_roll, curr_pitch;
  m.getRPY(curr_roll, curr_pitch, current_yaw);

  odom_received = true;

}

void MissionManager::timerClb(const ros::TimerEvent& event){

  if(!(odom_received)) return;

  bool position_control_granted;
  nh_.getParam("position_control_granted", position_control_granted);

  bool yaw_control_granted;
  nh_.getParam("yaw_control_granted", yaw_control_granted);

  if(position_control_granted && (min_dist_down < min_height)){

    ROS_WARN("Obstacle below the MAV");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    give_up_position_control_client_.call(give_up_position_control);
    nh_.getParam("position_control_granted", position_control_granted);

  }

  if (!position_control_granted){//stop all the behaviours

    if(performing_sweep){//pause the sweeping in course (if any)

      sweep_status = 2;
      nh_.setParam("sweep_status", sweep_status);
      ROS_WARN("Sweeping paused");
      performing_sweep = false;

      // save WP to allow resuming the sweeping
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;

      pausedMission_initial_yaw = initial_yaw_sweep;

      removeLastWPPath();
      addWP2WPPath(current_x, current_y, current_z);

    }

    if(performing_vinspection){//pause the vertical inspection in course (if any)

      vinspection_status = 2;
      nh_.setParam("vinspection_status", vinspection_status);
      ROS_WARN("Vertical inspection paused");
      performing_vinspection = false;

      // save WP to allow resuming the vertical inspection
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;

      pausedMission_initial_yaw = initial_yaw_vinspection;

      removeLastWPPath();
      addWP2WPPath(current_x, current_y, current_z);

    }

    if(performing_cinspection){//pause the circular inspection in course (if any)

      cinspection_status = 2;
      nh_.setParam("cinspection_status", cinspection_status);
      ROS_WARN("Circular inspection paused");
      performing_cinspection = false;

      // save WP to allow resuming the circular inspection
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;
      pausedMission_WP_yaw = WP_yaw;

      pausedMission_initial_yaw = initial_yaw_cinspection; //unnecessary for this behaviour

      removeLastWPPath();
      addWP2WPPath(current_x, current_y, current_z);

    }

    //stop all the other behaviours
    nh_.setParam("hovering", false);
    nh_.setParam("going_home", false);
    nh_.setParam("going_to_point", false);

  }

  if(!yaw_control_granted){//stop the yaw control

    if(performing_cinspection){//pause the circular inspection in course (if any)

      cinspection_status = 2;
      nh_.setParam("cinspection_status", cinspection_status);
      ROS_WARN("Circular inspection paused");
      performing_cinspection = false;

      // save WP to allow resuming the circular inspection
      pausedMission_WP_x = WP_x;
      pausedMission_WP_y = WP_y;
      pausedMission_WP_z = WP_z;
      pausedMission_WP_yaw = WP_yaw;

      pausedMission_initial_yaw = initial_yaw_cinspection; //unnecessary for this behaviour

      removeLastWPPath();
      addWP2WPPath(current_x, current_y, current_z);

    }

    //stop the orientation keeping
    nh_.setParam("keeping_orientation", false);
  }

  if(performing_sweep){

    performSweep(); //it provides the WP

  }

  if(performing_vinspection){

    performVerticalInspection(); //it provides the WP

  }

  if(performing_cinspection){

    performCircularInspection(); //it provides the WP

  }

  geometry_msgs::PosePtr pose_WP(new geometry_msgs::Pose);
  bool publish_WP = false;

  if(position_control_granted){ // perform some mission/behaviour

    // publish the current WP
    pose_WP->position.x = WP_x;
    pose_WP->position.y = WP_y;
    pose_WP->position.z = WP_z;
    pose_WP->orientation.w = 1.0;

    if(follow_trajectory){

      double errorX = WP_x-current_x;
      double errorY = WP_y-current_y;
      double errorZ = WP_z-current_z;
      double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

      if(errorWP > WP_tolerance){ //we are far from the WP

        int WPs_num = WP_path->poses.size();
        double prev_WP_x = WP_path->poses.at(WPs_num-2).pose.position.x;
        double prev_WP_y = WP_path->poses.at(WPs_num-2).pose.position.y;
        double prev_WP_z = WP_path->poses.at(WPs_num-2).pose.position.z;

        tf::Vector3 current_pose_vect = tf::Vector3(current_x, current_y, current_z);
        tf::Vector3 WP_vect = tf::Vector3(WP_x, WP_y, WP_z);
        tf::Vector3 prev_WP_vect = tf::Vector3(prev_WP_x, prev_WP_y, prev_WP_z);
        
        //compute the direction vector of the path between WPs
        double Ap = WP_x - prev_WP_x;
        double Bp = WP_y - prev_WP_y;
        double Cp = WP_z - prev_WP_z;
        tf::Vector3 WPs_dir_vect = tf::Vector3(Ap, Bp, Cp);

        //compute the distance from the current position to the line connecting both WPs
        double distToPath = (((current_pose_vect-prev_WP_vect).cross(WPs_dir_vect)).length())/(WPs_dir_vect.length());

        //get the plane (Ap*x + Bp*y + Cp*z + D = 0) containing the current pose and whose normal is the direction vector
        //of the line containing both WPs. The only missing term is D.
        double D = -(Ap*current_x + Bp*current_y + Cp*current_z);

        //compute the point C being point in the path which is the closest to the current position
        //  the equation of the line connecting both WPs is: 
        //  (x,y,z) = (prev_WP_x,prev_WP_y,prev_WP_z) + lambda*(Ap,Bp,Cp)
        double lambda = (Ap*prev_WP_x + Bp*prev_WP_y + Cp*prev_WP_z + D) / -(Ap*Ap + Bp*Bp + Cp*Cp);
        double closest_x = prev_WP_x + lambda * Ap;
        double closest_y = prev_WP_y + lambda * Bp;
        double closest_z = prev_WP_z + lambda * Cp;
        tf::Vector3 closest_point_vect = tf::Vector3(closest_x, closest_y, closest_z);

        double distToPrevWP = prev_WP_vect.distance(closest_point_vect); // distance from C to the previous WP
        double distToWP = WP_vect.distance(closest_point_vect); //distance from C to the current WP
        double distWPs = prev_WP_vect.distance(WP_vect); //distance between WPs

        
        if(((distWPs > distToPrevWP) && (distWPs > distToWP)) || (distToPrevWP < WP_tolerance)){//if C is between the two WPs or C is very close to the previous_WP

          if(distToPath > follow_trajectory_lambda){ //distToPath is larger than the follow_trajectory_lambda then
            // we are far from the path --> go back to the path
            // the WP is set to C
            pose_WP->position.x = closest_x;
            pose_WP->position.y = closest_y;
            pose_WP->position.z = closest_z;

          }else{ // we are close to the path
            // the WP is set to C + a vector pointing to the original WP with length (follow_trajectory_delta - distToPath)
            // tf::Vector3 VTP = closest_point_vect + (WPs_dir_vect.normalize()*(follow_trajectory_delta - distToPath));
            // the WP is set to C + a vector pointing to the original WP with length (follow_trajectory_delta * (1-distToPath/follow_trajectory_lambda))
            tf::Vector3 VTP = closest_point_vect + (WPs_dir_vect.normalize()*(follow_trajectory_delta * (1-(distToPath/follow_trajectory_lambda))));

            double distToVTP = VTP.distance(closest_point_vect); //distance from C to the VTP
            
            if(distToVTP < distToWP){ // the VTP is closer than the WP
              pose_WP->position.x = VTP.getX();
              pose_WP->position.y = VTP.getY();
              pose_WP->position.z = VTP.getZ();
            }
          }

          //ROS_WARN("WP: %2.2f, %2.2f, %2.2f", WP_x, WP_y, WP_z);
          //ROS_WARN("prev_WP: %2.2f, %2.2f, %2.2f", prev_WP_x, prev_WP_y, prev_WP_z);
          //ROS_WARN("VTP: %2.2f, %2.2f, %2.2f", pose_WP->position.x, pose_WP->position.y, pose_WP->position.z);
          //ROS_WARN("-----------");
        }else if(distToWP > distToPrevWP){//if C is not between the two WPs and C is closer to the previous WP
          // go to the previous WP
          pose_WP->position.x = prev_WP_x;
          pose_WP->position.y = prev_WP_y;
          pose_WP->position.z = prev_WP_z;
          
          //ROS_WARN("WP: %2.2f, %2.2f, %2.2f", WP_x, WP_y, WP_z);
          //ROS_WARN("prev_WP: %2.2f, %2.2f, %2.2f", prev_WP_x, prev_WP_y, prev_WP_z);
          //ROS_WARN("VTP: %2.2f, %2.2f, %2.2f", pose_WP->position.x, pose_WP->position.y, pose_WP->position.z);
          //ROS_WARN("-----------");

        }//otherwise the WP is not modified

      }
    }

    //pose_pub_.publish(pose_WP);
    publish_WP = true;

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

  if(yaw_control_granted){

    // publish the WP yaw
    tf::Quaternion q;
    q.setRPY(0,0,WP_yaw);
    tf::quaternionTFToMsg(q, pose_WP->orientation);

    publish_WP = true;

    if(!yaw_controller_enabled){

      //enable yaw control
      srv_mav_control::EnableYawControl enable_yaw_ctrl;
      enable_yaw_ctrl.request.enable = 1;
      enable_yaw_control_client_.call(enable_yaw_ctrl);

      if(enable_yaw_ctrl.response.enabled){
        yaw_controller_enabled = true;
      }

    }

  }else{ // do not perform yaw control

    if(yaw_controller_enabled){

      //disable yaw_control
      srv_mav_control::EnableYawControl enable_yaw_ctrl;
      enable_yaw_ctrl.request.enable = 0;
      enable_yaw_control_client_.call(enable_yaw_ctrl);

      if(!enable_yaw_ctrl.response.enabled){
        yaw_controller_enabled = false;
      }
    }

  }

  if(publish_WP){
    pose_pub_.publish(pose_WP);
    publishWPPath();
  }

  if(publish_mission_path){
    if ((sweep_status > 0)||(vinspection_status > 0)||(cinspection_status > 0)){//sweeping/vinspection/cinspection in progress or paused
      publishMissionPath();
    }else{
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

  if(errorWP < WP_tolerance){ // the WP has been reached

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

        // if(lateral_dist < (sweep_min_lateral_dist + WP_tolerance)) { // wall found

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

        ROS_WARN("Sweep finished");

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

    if(performing_sweep){// a new WP has been defined and the SW has not finished
      addWP2WPPath(WP_x, WP_y, WP_z);
    }

  } 

  if(!performing_sweep){ // the sweeping has finished now

    // give up control to the Safety Manager
    // srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    // give_up_position_control_client_.call(give_up_position_control);

    // start a hovering in the last WP instead of giving up control
    nh_.setParam("hovering", true);
    ROS_WARN("Hovering at %2.2f, %2.2f, %2.2f", WP_x, WP_y, WP_z);

  }

}

void MissionManager::performVerticalInspection(){

  double errorX = WP_x-current_x;
  double errorY = WP_y-current_y;
  double errorZ = WP_z-current_z;

  double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

  if(errorWP < WP_tolerance){ // the WP has been reached

    // update the vinspection_state if necessary

    if((vinspection_state == 0) || (vinspection_state == 2)){ // going up or down

      vinspection_z_accumulated += vinspection_z_increment;

      if(!vinspection_to_ceiling){

        if (vinspection_z_accumulated >= vinspection_z_size){ // vertical movement finished

          vinspection_state ++;
          vinspection_state = vinspection_state%3;
          vinspection_z_accumulated = 0.0;

          if (vinspection_state == 0){ // the state says to go up again
            
            ROS_WARN("Vertical inspection finished");

            performing_vinspection = false;
            vinspection_status = 0;
            nh_.setParam("vinspection_status", vinspection_status);

          }

        } //else: keep going in that direction

      }else{// vertical inspection up-to-ceiling

        // if(((vinspection_state == 0) && (min_dist_up < (vinspection_min_ceiling_dist + WP_tolerance))) || 
        //     ((vinspection_state == 2) && (current_z <= final_z_vinspection))){

        if(vinspection_reaching_end){

          vinspection_state ++;
          vinspection_state = vinspection_state%3;
          vinspection_z_accumulated = 0.0;

          if (vinspection_state == 0){ // the state says to go up again
            
            ROS_WARN("Vertical inspection finished");

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

    if(performing_vinspection){ // a new WP has been defined and the VI has not finished
      addWP2WPPath(WP_x, WP_y, WP_z);
    }

  } 

  if(!performing_vinspection){ // the vertical inspection has finished now

    // give up control to the Safety Manager
    // srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    // give_up_position_control_client_.call(give_up_position_control);

    // start a hovering in the last WP instead of giving up control
    nh_.setParam("hovering", true);
    ROS_WARN("Hovering at %2.2f, %2.2f, %2.2f", WP_x, WP_y, WP_z);

  }

}

void MissionManager::performCircularInspection(){


  //always update the WP_yaw to look at the center of the circumference
  double center_errorX = center_x_cinspection-current_x;
  double center_errorY = center_y_cinspection-current_y;
  // double dist_to_center = sqrt(center_errorX*center_errorX + center_errorY*center_errorY);
  // double yaw_incr = acos((cos(current_yaw)*center_errorX + sin(current_yaw)*center_errorY)/ dist_to_center);
  // double delta_aux = cos(current_yaw)*center_errorY - sin(current_yaw)*center_errorX;
  // if (delta_aux < 0.0) yaw_incr = -yaw_incr;
  // WP_yaw = current_yaw + yaw_incr;
  WP_yaw = atan2(center_errorY, center_errorX);

  // update the cinspection_state if necessary
  double angular_diff = acos(cos(WP_yaw)*cos(initial_yaw_cinspection) + sin(WP_yaw)*sin(initial_yaw_cinspection));

  if((cinspection_state == 0) && (angular_diff > M_PI_2)) cinspection_state = 1; // first 90 deg. completed
  else if((cinspection_state == 1) && (angular_diff < 0.02)) cinspection_state = 2; // circumference completed
  
  double errorX = WP_x-current_x;
  double errorY = WP_y-current_y;
  double errorZ = WP_z-current_z;

  double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

  if(errorWP < WP_tolerance){ // the WP has been reached

    if( cinspection_state < 2){

      //compute the next WP coordinates
      //compute the displacement
      double theta = cinspection_arc_increment/cinspection_radius; // L = theta * radius ; then theta = L/r
      double alpha = (M_PI-theta)/2.0;
      double beta = M_PI_2 - alpha;
      double d = sqrt(2*cinspection_radius*cinspection_radius * (1-cos(theta)));

      tf::Vector3 robot_incr(d*sin(beta), -d*cos(beta), 0.0); //move following a circumference

      //rotate the increment to the world frame using the estimated yaw
      tf::Matrix3x3 m_rot;

      center_errorX = center_x_cinspection-WP_x;
      center_errorY = center_y_cinspection-WP_y;
      // yaw_incr = acos((cos(current_yaw)*center_errorX + sin(current_yaw)*center_errorY)/ cinspection_radius);
      // delta_aux = cos(current_yaw)*center_errorY - sin(current_yaw)*center_errorX;
      // if (delta_aux < 0.0) yaw_incr = -yaw_incr;
      // double yaw_to_center = current_yaw + yaw_incr;
      double yaw_to_center = atan2(center_errorY, center_errorX);
      m_rot.setRPY(0, 0, yaw_to_center); // use the yaw to look to the center from the last WP reached
      tf::Vector3 world_incr = m_rot * robot_incr;

      //compute the next WP
      WP_x = WP_x + world_incr.getX();
      WP_y = WP_y + world_incr.getY();

    }else{

      ROS_WARN("Circular inspection finished");

      performing_cinspection = false;
      cinspection_status = 0;
      nh_.setParam("cinspection_status", cinspection_status);

    }

    if(performing_cinspection){ // a new WP has been defined and the CI has not finished
      addWP2WPPath(WP_x, WP_y, WP_z);
    }
  }

  if(!performing_cinspection){ // the circular inspection has finished now

    // give up control to the Safety Manager
    // srv_mav_behaviours::GiveUpPositionControl give_up_position_control;
    // give_up_position_control_client_.call(give_up_position_control);

    // start a hovering in the last WP instead of giving up control
    nh_.setParam("hovering", true);
    ROS_WARN("Hovering at %2.2f, %2.2f, %2.2f", WP_x, WP_y, WP_z);

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

  mission_path->header.frame_id = world_frame;
  mission_path->header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = mission_path->header.stamp;
  point.pose.orientation.w = 1.0;

  //add current position as initial point
  point.pose.position.x = current_x;
  point.pose.position.y = current_y;
  point.pose.position.z = current_z;
  mission_path->poses.push_back(point);

  tf::Vector3 robot_incr(0.0, -sweep_y_size, 0.0);
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

      point.pose.position.x = other_x;
      point.pose.position.y = other_y;

    }else{ // go back to the left

      point.pose.position.x = current_x;
      point.pose.position.y = current_y;

    }

    point.pose.position.z = last_z;
    right = !right;

    mission_path->poses.push_back(point); // add next point to the right or left

    next_z = last_z - sweep_z_increment; 

    if(next_z < final_z) break; // beyond sweeping vertical limit 
    else if(next_z < min_height) break; // too close to the ground
    else{

      point.pose.position.z = next_z;
      last_z = next_z;

      mission_path->poses.push_back(point); // add next point going down

    }
  }
}

void MissionManager::recomputeSweepingPath(){

  double first_x = mission_path->poses.at(0).pose.position.x;
  double first_y = mission_path->poses.at(0).pose.position.y;
  double first_z = mission_path->poses.at(0).pose.position.z;
  tf::Vector3 first_point(first_x, first_y, first_z);
  tf::Matrix3x3 mat;
  mat.setRPY(0, 0, initial_yaw_sweep-pausedMission_initial_yaw); //update in SW orientation
  tf::Vector3 pausedMission_WP(pausedMission_WP_x, pausedMission_WP_y, pausedMission_WP_z);
  pausedMission_WP = pausedMission_WP - first_point;
  tf::Vector3 pausedMission_WP_rot = mat * pausedMission_WP;
  tf::Vector3 WP(WP_x, WP_y, WP_z);
  tf::Vector3 offsetWPs = WP - pausedMission_WP_rot;

  int num_points = mission_path->poses.size();
  double point_x, point_y, point_z;
  for(int i = 0; i < num_points; i++){
    point_x = mission_path->poses.at(i).pose.position.x;
    point_y = mission_path->poses.at(i).pose.position.y;
    point_z = mission_path->poses.at(i).pose.position.z;
    tf::Vector3 point(point_x, point_y, point_z);
    point = point - first_point;
    point = mat * point;
    point = point + offsetWPs;
    mission_path->poses.at(i).pose.position.x = point.getX();
    mission_path->poses.at(i).pose.position.y = point.getY();
    mission_path->poses.at(i).pose.position.z = point.getZ();
  }

}

void MissionManager::createVerticalInspectionPath(){

  clearMissionPath();

  mission_path->header.frame_id = world_frame;
  mission_path->header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = mission_path->header.stamp;
  point.pose.orientation.w = 1.0;

  //the path comprises just four points

  //add current position as initial point
  point.pose.position.x = current_x;
  point.pose.position.y = current_y;
  point.pose.position.z = current_z;
  mission_path->poses.push_back(point);

  tf::Vector3 robot_incr(0.0, -vinspection_y_size, 0.0);
  tf::Matrix3x3 m_rot;
  m_rot.setRPY(0, 0, initial_yaw_vinspection);
  tf::Vector3 world_incr = m_rot * robot_incr;
  double other_x = current_x + world_incr.getX();
  double other_y = current_y + world_incr.getY();

  //add second point
  point.pose.position.z = current_z + vinspection_z_size;
  mission_path->poses.push_back(point);

  //add third point
  point.pose.position.x = other_x;
  point.pose.position.y = other_y;
  mission_path->poses.push_back(point);

  //add last point
  point.pose.position.z = current_z;
  mission_path->poses.push_back(point);

}

void MissionManager::recomputeVerticalInspectionPath(){

  double first_x = mission_path->poses.at(0).pose.position.x;
  double first_y = mission_path->poses.at(0).pose.position.y;
  double first_z = mission_path->poses.at(0).pose.position.z;
  tf::Vector3 first_point(first_x, first_y, first_z);
  tf::Matrix3x3 mat;
  mat.setRPY(0, 0, initial_yaw_vinspection-pausedMission_initial_yaw); //update in VI orientation
  tf::Vector3 pausedMission_WP(pausedMission_WP_x, pausedMission_WP_y, pausedMission_WP_z);
  pausedMission_WP = pausedMission_WP - first_point;
  tf::Vector3 pausedMission_WP_rot = mat * pausedMission_WP;
  tf::Vector3 WP(WP_x, WP_y, WP_z);
  tf::Vector3 offsetWPs = WP - pausedMission_WP_rot;

  int num_points = mission_path->poses.size();
  double point_x, point_y, point_z;
  for(int i = 0; i < num_points; i++){
    point_x = mission_path->poses.at(i).pose.position.x;
    point_y = mission_path->poses.at(i).pose.position.y;
    point_z = mission_path->poses.at(i).pose.position.z;
    tf::Vector3 point(point_x, point_y, point_z);
    point = point - first_point;
    point = mat * point;
    point = point + offsetWPs;
    mission_path->poses.at(i).pose.position.x = point.getX();
    mission_path->poses.at(i).pose.position.y = point.getY();
    mission_path->poses.at(i).pose.position.z = point.getZ();
  }

}

void MissionManager::createCircularInspectionPath(){

  clearMissionPath();

  mission_path->header.frame_id = world_frame;
  mission_path->header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = mission_path->header.stamp;
  point.pose.orientation.w = 1.0;

  int num_points = 200;
  double theta_incr = 2*M_PI/num_points;
  double theta = 0.0;

  for(int i = 0; i <= num_points; i++){
    point.pose.position.x = center_x_cinspection + cinspection_radius*cos(theta);
    point.pose.position.y = center_y_cinspection + cinspection_radius*sin(theta);
    point.pose.position.z = current_z;
    theta += theta_incr;
    mission_path->poses.push_back(point);
  }

}

void MissionManager::recomputeCircularInspectionPath(){

  // we can draw the circumference from scratch
  // the new radius will be used (if updated)
  createCircularInspectionPath();

}

void MissionManager::publishWPPath(){

  WP_path_pub_.publish(WP_path);

}

void MissionManager::newWPPath(){

  WP_path = nav_msgs::PathPtr(new nav_msgs::Path);

  WP_path->header.frame_id = world_frame;
  WP_path->header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = ros::Time::now();
  point.pose.orientation.w = 1.0;

  point.pose.position.x = current_x;
  point.pose.position.y = current_y;
  point.pose.position.z = current_z;
  WP_path->poses.push_back(point);

}

void MissionManager::addWP2WPPath(double x, double y, double z){

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = ros::Time::now();
  point.pose.orientation.w = 1.0;

  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  WP_path->poses.push_back(point);

}

void MissionManager::removeLastWPPath(){

  WP_path->poses.pop_back();

}

}  // namespace srv_mav_behaviours
