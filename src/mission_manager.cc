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

  if(!performing_sweep){

    // sweep_y_size = config.sweep_y_size;
    // sweep_z_size = config.sweep_z_size;
    // sweep_z_increment = config.sweep_z_increment;
    sweep_WP_error = config.sweep_WP_error;

  }else{
    ROS_WARN("Sweeping parameters can not be modified now");
  }


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

  // nh_.param("sweep_y_size", sweep_y_size, 3.0);
  // ROS_INFO("Sweep_y_size: %2.2f", sweep_y_size);

  // nh_.param("sweep_z_size", sweep_z_size, 3.0);
  // ROS_INFO("Sweep_z_size: %2.2f", sweep_z_size);

  // nh_.param("sweep_z_increment", sweep_z_increment, 1.0);
  // ROS_INFO("Sweep_z_increment: %2.2f", sweep_z_increment);

  nh_.param("sweep_WP_error", sweep_WP_error, 0.3);
  ROS_INFO("Sweep_WP_error: %2.2f", sweep_WP_error);

  checkParameters();

  odometry_received = false;

  position_control_granted = false;
  nh_.setParam("position_control_granted", false); //set to false the parameter safetyManager/position_control_granted (also done by the safetyManager)

  position_controllers_enabled = false;

  performing_sweep = false;

  WP_x = WP_y = WP_z = 0.0;
  initial_yaw_sweep = 0.0;
  final_z_sweep = 0.0;
  sweep_state = 0;  //0--> go right
                    //1--> go down
                    //2--> go left
                    //3--> go down

  sweep_status = 0; //0-->No sweeping in course
                    //1-->Sweeping
                    //2-->Paused
  nh_.setParam("sweep_status", sweep_status);

  sweep_y_size = sweep_z_size = sweep_z_increment = 0.0;

  // Publishers
  pose_pub_ = nh_.advertise<geometry_msgs::Pose>("way_point", 1);

  // Subscribers
  odom_subs_ = nh_.subscribe("odometry", 1, &MissionManager::odomClb, this);

  // Advertising Services
  start_sweep_srv_ = nh_.advertiseService("start_sweep", &MissionManager::startSweep, this);
  stop_sweep_srv_ = nh_.advertiseService("stop_sweep", &MissionManager::stopSweep, this);
  pause_sweep_srv_ = nh_.advertiseService("pause_sweep", &MissionManager::pauseSweep, this);
  resume_sweep_srv_ = nh_.advertiseService("resume_sweep", &MissionManager::resumeSweep, this);

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
  // sweep_y_size = abs(sweep_y_size);
  // sweep_z_size = abs(sweep_z_size);
  // sweep_z_increment = abs(sweep_z_increment);
  sweep_WP_error = abs(sweep_WP_error);

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

}

bool MissionManager::startSweep(srv_mav_behaviours::StartSweep::Request &req, srv_mav_behaviours::StartSweep::Response &res){

  if(!odometry_received) return false;

  if(performing_sweep || (sweep_status == 2)){
    ROS_WARN("Sweep already in process!!");
    return false;
  }

  if(current_z < min_height){
    ROS_WARN("Flying too low to start a sweep");
    return false;
  }

  if(current_z > max_height){
    ROS_WARN("Flying too high to start a sweep");
    return false;
  }

  // request control to the Safety Manager
  srv_mav_behaviours::RequestControl request_control;
  request_control_client_.call(request_control);

  if(request_control.response.allowed){ // start the sweep

    sweep_y_size = req.sweep_y_size;
    sweep_z_size = req.sweep_z_size;
    sweep_z_increment = req.sweep_z_increment;

    position_control_granted = true;

    //the sweep starts from the top left corner

    //compute the first displacement
    tf::Vector3 robot_incr(0.0, -sweep_y_size, 0.0); //move to the right

     //rotate the increment to the world frame using the estimated yaw
    tf::Matrix3x3 m_rot;
    m_rot.setRPY(0, 0, current_yaw);
    tf::Vector3 world_incr = m_rot * robot_incr;

    //compute the first WP
    WP_x = current_x + world_incr.getX();
    WP_y = current_y + world_incr.getY();
    WP_z = current_z;
    initial_yaw_sweep = current_yaw;
    final_z_sweep = current_z - sweep_z_size;

    performing_sweep = true;  
    sweep_status = 1;
    nh_.setParam("sweep_status", sweep_status);
  
    sweep_state = 0; // goint to the right
    ROS_WARN("Starting new sweep");

  }else{ // there is probably another mission in progress

    ROS_WARN("New sweep not allowed");

  }

  return true;
}

bool MissionManager::stopSweep(srv_mav_behaviours::StopSweep::Request &req, srv_mav_behaviours::StopSweep::Response &res){

  if(performing_sweep){

    performing_sweep = false;
    sweep_status = 0;
    nh_.setParam("sweep_status", sweep_status);
    ROS_WARN("Sweeping stopped");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);
    position_control_granted = false;

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

bool MissionManager::pauseSweep(srv_mav_behaviours::PauseSweep::Request &req, srv_mav_behaviours::PauseSweep::Response &res){

  if(performing_sweep){

    performing_sweep = false;
    sweep_status = 2;
    nh_.setParam("sweep_status", sweep_status);
    ROS_WARN("Sweeping paused");

    // give up control to the Safety Manager
    srv_mav_behaviours::GiveUpControl give_up_control;
    give_up_control_client_.call(give_up_control);
    position_control_granted = false;

  }else{

    ROS_WARN("No sweeping in course");

  }

  return true;
}

bool MissionManager::resumeSweep(srv_mav_behaviours::ResumeSweep::Request &req, srv_mav_behaviours::ResumeSweep::Response &res){

  if(sweep_status == 2){ // the last sweeping is paused

    // request control to the Safety Manager
    srv_mav_behaviours::RequestControl request_control;
    request_control_client_.call(request_control);

    if(request_control.response.allowed){ // resume the sweep

      position_control_granted = true;

      //recompute the WP with the current orientation

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

    }else{ // there is probably another mission in progress

      ROS_WARN("Resume sweep not allowed");

    }

  }else{

    ROS_WARN("No sweeping in pause");

  }

  return true;
}

void MissionManager::odomClb(const nav_msgs::Odometry::ConstPtr& odo_msg){

  current_x = odo_msg->pose.pose.position.x;
  current_y = odo_msg->pose.pose.position.y;
  current_z = odo_msg->pose.pose.position.z;

  tf::Quaternion q;
  tf::quaternionMsgToTF(odo_msg->pose.pose.orientation, q);
  tf::Matrix3x3 m(q);
  double curr_roll, curr_pitch;
  m.getRPY(curr_roll, curr_pitch, current_yaw);

  odometry_received = true;

}

void MissionManager::timerClb(const ros::TimerEvent& event){

  if(!(odometry_received)) return;

  nh_.getParam("position_control_granted", position_control_granted);

  if (!position_control_granted){//stop all the behaviours

    if(performing_sweep){//pause the sweeping in course (if any)
      sweep_status = 2;
      nh_.setParam("sweep_status", sweep_status);
      ROS_WARN("Sweeping paused");
    }

    performing_sweep = false;
  }

  if(performing_sweep){

    performSweep();

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

}

void MissionManager::performSweep(){

  double errorX = WP_x-current_x;
  double errorY = WP_y-current_y;
  double errorZ = WP_z-current_z;

  double errorWP = sqrt(errorX*errorX + errorY*errorY + errorZ*errorZ);

  if(errorWP < sweep_WP_error){ // compute the next WP

    sweep_state ++;
    sweep_state = sweep_state%4;

    if((sweep_state == 1) || (sweep_state == 3)){ // go down

      double aux_WP_z = WP_z - sweep_z_increment;

      if(aux_WP_z < min_height){

        ROS_WARN("Flying too low to keep on sweeping");

        performing_sweep = false;
        sweep_status = 0;
        nh_.setParam("sweep_status", sweep_status);

      }else if(aux_WP_z < final_z_sweep){

        ROS_WARN("Sweep finised");

        performing_sweep = false;
        sweep_status = 0;
        nh_.setParam("sweep_status", sweep_status);

      }else{

        WP_z = aux_WP_z; // we update WPz only if it is reachable 

      }

    }else{ // go to the right or to the left

      double robot_incr_y;

      if(sweep_state == 0){ // go to the right

        robot_incr_y = -sweep_y_size;

      }else{ //go to the left (sweep_state == 2)

        robot_incr_y = sweep_y_size;
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
    position_control_granted = false;

  }

}

}  // namespace srv_mav_behaviours
