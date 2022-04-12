//
// Created by smurolap on 2022/2/10.
//

#ifndef SMURO_DEV_RATBOT_SMURO_IMITATION_INCLUDE_SYCHROCONTROLLER_SYCHROCONTROLLER_H_
#define SMURO_DEV_RATBOT_SMURO_IMITATION_INCLUDE_SYCHROCONTROLLER_SYCHROCONTROLLER_H_
/*************************************************************
 * File Name : synchrocontroller.h
 * Author    : ZEFFIRETTI, HESH
 * College   : Beijing Institute of Technology
 * E-Mail    : zeffiretti@bit.edu.cn, hiesh@mail.com
**********************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <smuro_msgs/Command.h>
#include <smuro_msgs/Pose.h>
#include <smuro_msgs/operators.h>
#include <smuro_io/filereader/textreader.h>
#include <smuro/smuro_robotics/smuro_robotics.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <experimental/filesystem>
#include <cmath>
#include <algorithm>
#include <bits/stdc++.h>
//#include <lis>

class SynchroController {
 public:
  SynchroController(ros::NodeHandle *nh, ros::NodeHandle *pnh);
  ~SynchroController() = default;
  void controlLoop(const ros::TimerEvent &evnet);

 private:
  ros::NodeHandle nh_;
  ros::Timer loop_timer;
  int frequency = 200;
  ros::Publisher demo_command_pub_;
  ros::Publisher demo_pose_pub_;
  ros::Publisher policy_command_pub_;
  ros::Publisher policy_pose_pub_;
  smuro_msgs::Pose demo_pose_;
  smuro_msgs::Pose pre_demo_pose_;
  smuro_msgs::Command demo_command_;
  smuro_msgs::Pose policy_pose_;
  smuro_msgs::Pose pre_policy_pose_;
  smuro_msgs::Command policy_command_;
  Eigen::VectorXd demo_joints;
  Eigen::VectorXd pre_demo_joints;
  Eigen::VectorXd policy_joints;
  Eigen::VectorXd pre_policy_joints;
  Eigen::MatrixXd data;
  int index = 0;
  double low_pass_filter_alpha = 0.0;
  std::string file_path;
  std::string demo_rat = "demo_rat", policy_robot = "policy_robot";
  SmuroRobotics *smuro_robotics_;

  // parameters used to control policy robot
  double MINIMAL_DISTANCE = 0.05;

  // private methods
  // distance between two points (x1,y1) and (x2,y2) with type of template T
  template<typename T>
  T distance(T x1, T y1, T x2, T y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  // angles of vector (x1,y1) and (x2,y2) with type of template T from X axis
  // return a pair variable (cos, sin)
  template<typename T>
  std::pair<T, T> angle(T x1, T y1, T x2, T y2) {
    T cos_angle, sin_angle;
    cos_angle = (x1 * x2 + y1 * y2) / (sqrt(x1 * x1 + y1 * y1) * sqrt(x2 * x2 + y2 * y2));
    sin_angle = (x1 * y2 - x2 * y1) / (sqrt(x1 * x1 + y1 * y1) * sqrt(x2 * x2 + y2 * y2));
    return std::make_pair(cos_angle, sin_angle);
  }
};

#endif //SMURO_DEV_RATBOT_SMURO_IMITATION_INCLUDE_SYCHROCONTROLLER_SYCHROCONTROLLER_H_
