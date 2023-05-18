/*
* This file is part of srv_mav_behaviours.
*
* Copyright (C) 2023 Francisco bonnin-Pascual <xisco.bonnin@uib.es> (University of the Balearic Islands)
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

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <srv_mav_behaviours/StartFollowPath.h>

ros::ServiceClient start_follow_path_client_;
ros::ServiceServer service_caller_srv_;
nav_msgs::Path demo_path;

bool callFollowPath(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

  srv_mav_behaviours::StartFollowPath start_follow_path;
  start_follow_path.request.path = demo_path;
  start_follow_path_client_.call(start_follow_path);

  return true;

}

int main(int argc, char** argv) {

  ros::init (argc, argv, "path_demo");
  ros::NodeHandle nh("~");

  start_follow_path_client_ = nh.serviceClient<srv_mav_behaviours::StartFollowPath>("/mission_manager/start_follow_path");
  service_caller_srv_ = nh.advertiseService("call_follow_path", &callFollowPath);

  std::string world_frame = "map";

  demo_path.header.frame_id = world_frame;
  demo_path.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped point;

  point.header.frame_id = world_frame;
  point.header.stamp = demo_path.header.stamp;
  point.pose.orientation.w = 1.0;

  int num_points = 10;
  double theta_incr = 2*M_PI/num_points;
  double theta = 0.0;

  double cinspection_radius = 4;
  double center_x_cinspection = -cinspection_radius;
  double center_y_cinspection = 0.0;
  double height = 2.5;

  for(int i = 0; i <= num_points; i++){
    point.pose.position.x = center_x_cinspection + cinspection_radius*cos(theta);
    point.pose.position.y = center_y_cinspection + cinspection_radius*sin(theta);
    point.pose.position.z = height;
    theta += theta_incr;
    demo_path.poses.push_back(point);
  }

  ros::spin();
  return 0;
}
