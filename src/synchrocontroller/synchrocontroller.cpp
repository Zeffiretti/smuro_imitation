//
// Created by smurolap on 2022/2/10.
//

/*************************************************************
 * File Name : synchrocontroller.cpp
 * Author    : ZEFFIRETTI, HESH
 * College   : Beijing Institute of Technology
 * E-Mail    : zeffiretti@bit.edu.cn, hiesh@mail.com
**********************************************************/

#include "smuro_imitation/synchrocontroller//synchrocontroller.h"

SynchroController::SynchroController(ros::NodeHandle *nh, ros::NodeHandle *pnh) {
  std::string cmd_topic = pnh->param("command_topic", std::string("motion/command"));
  std::string demo_cmd_topic = "/" + demo_rat + "/" + cmd_topic;
  std::string demo_pose_topic = demo_cmd_topic + "/pose";
  std::string policy_cmd_topic = "/" + policy_robot + "/" + cmd_topic;
  std::string policy_pose_topic = policy_cmd_topic + "/pose";
  // get index from ros param start_index
  index = pnh->param("start_index", 0);
  frequency = pnh->param("frequency", 200);
  low_pass_filter_alpha = pnh->param("low_pass_filter_alpha", 0.99);
  MINIMAL_DISTANCE = pnh->param("minimal_distance", 0.1);
  // publisher configuration
  demo_command_pub_ = nh->advertise<smuro_msgs::Command>(demo_cmd_topic, 1);
  demo_pose_pub_ = nh->advertise<smuro_msgs::Pose>(demo_pose_topic, 1);
  policy_command_pub_ = nh->advertise<smuro_msgs::Command>(policy_cmd_topic, 1);
  policy_pose_pub_ = nh->advertise<smuro_msgs::Pose>(policy_pose_topic, 1);

  // read data from txt file
  auto default_path = ros::package::getPath("smuro_imitation") + "/data/data.txt";
  file_path = pnh->param("file_path", default_path);
  if (file_path == "none") {
    file_path = default_path;
  }
  // check whether the file exists
  if (!std::ifstream(file_path)) {
    ROS_ERROR("File %s does not exist!", file_path.c_str());
    ros::shutdown();
  }

  data = smuro_io::readMatrixFromTxt(file_path);

  pre_demo_pose_.x = data(index, 5);
  pre_demo_pose_.y = data(index, 6);
  pre_demo_pose_.yaw = data(index, 7);
  pre_policy_pose_.x = data(index, 13);
  pre_policy_pose_.y = data(index, 14);
  pre_policy_pose_.yaw = data(index, 15);

  // print the line
  smuro_robotics_ = new SmuroRobotics;
  // print the line

  demo_joints = Eigen::VectorXd::Zero(5);
  pre_demo_joints = Eigen::VectorXd::Zero(5);
  policy_joints = Eigen::VectorXd::Zero(5);
  pre_policy_joints = Eigen::VectorXd::Zero(5);
  std::cout << "processing to " << __FILE__ << " at " << __LINE__ << std::endl;

  loop_timer = nh->createTimer(ros::Duration(1.0 / frequency),
                               &SynchroController::controlLoop,
                               this);
}

void SynchroController::controlLoop(const ros::TimerEvent &evnet) {
  // configure demo pose
//  demo_pose_.x = data(index, 5) * 1.5;
//  demo_pose_.y = data(index, 6) * 1.5;
  demo_pose_.x = data(index, 5);
  demo_pose_.y = data(index, 6);
  demo_pose_.yaw = data(index, 7);
  demo_pose_.x = low_pass_filter_alpha * pre_demo_pose_.x + (1 - low_pass_filter_alpha) * demo_pose_.x;
  demo_pose_.y = low_pass_filter_alpha * pre_demo_pose_.y + (1 - low_pass_filter_alpha) * demo_pose_.y;
  demo_pose_.yaw = low_pass_filter_alpha * pre_demo_pose_.yaw + (1 - low_pass_filter_alpha) * demo_pose_.yaw;
  pre_demo_pose_ = demo_pose_;
  // configure demo command
//  demo_joints(0) = std::min(abs((data(index, 0) + data(index, 1)) * 0.3), 0.3);
//  demo_joints(1) =
////    std::min(-float(data(index, 0) * 0.7 + data(index, 1) + 0.1), -float(demo_joints(0) / 2));
//  demo_joints(0) = std::min(abs(data(index, 0) * 0.3), 0.5);
//  demo_joints(1) =
//    std::min(-float(data(index, 0) * 0.7 + data(index, 1) + 0.1), -float(demo_joints(0)));
  demo_joints(0) = data(index, 0) - 0.4;
  demo_joints(1) = data(index, 1);
  demo_joints(2) = data(index, 2);
  demo_joints(3) = data(index, 3);
  demo_joints(4) = data(index, 4);
  demo_joints = low_pass_filter_alpha * pre_demo_joints + (1 - low_pass_filter_alpha) * demo_joints;
  pre_demo_joints = demo_joints;
//  demo_command_.joints = smuro_msgs::jointPosFromEigen(smuro_robotics_->LimitJoint(demo_joints));
  demo_command_.joints = smuro_msgs::jointPosFromEigen(demo_joints);
//  // configure policy pose
//  policy_pose_.x = data(index, 13) * 1.5;
//  policy_pose_.y = data(index, 14) * 1.5;
//  policy_pose_.yaw = data(index, 15);

  policy_pose_.x = data(index, 13);
  policy_pose_.y = data(index, 14);
  policy_pose_.yaw = data(index, 15);
//  if (index > 40000) {
//    policy_pose_.yaw *= -1;
//  }
  policy_pose_.x = low_pass_filter_alpha * pre_policy_pose_.x + (1 - low_pass_filter_alpha) * policy_pose_.x;
  policy_pose_.y = low_pass_filter_alpha * pre_policy_pose_.y + (1 - low_pass_filter_alpha) * policy_pose_.y;
  policy_pose_.yaw = low_pass_filter_alpha * pre_policy_pose_.yaw + (1 - low_pass_filter_alpha) * policy_pose_.yaw;
  pre_policy_pose_ = policy_pose_;
  // ensure two bots are far enough
  if (distance(demo_pose_.x, demo_pose_.y, policy_pose_.x, policy_pose_.y) < MINIMAL_DISTANCE) {
    auto direction = angle(demo_pose_.x, demo_pose_.y, policy_pose_.x, policy_pose_.y);
    policy_pose_.x = demo_pose_.x + MINIMAL_DISTANCE * direction.first;
    policy_pose_.y = demo_pose_.y + MINIMAL_DISTANCE * direction.second;
  }
  //configure policy command
//  policy_joints(0) = std::min(abs((data(index, 8) + data(index, 9)) * 0.3), 0.3);
//  policy_joints(1) =
//    std::min(-float(data(index, 8) * 0.7 + data(index, 9) + 0.1), -float(policy_joints(0) / 2));
//  policy_joints(0) = std::min(abs(data(index, 8) * 0.3), 0.5);
//  policy_joints(1) = std::min(-float(data(index, 9) * 0.7 + data(index, 10) + 0.1), -float(policy_joints(0)));
//  policy_joints(2) = -data(index, 10) * 0.5;
//  policy_joints(3) = data(index, 11);
//  policy_joints(4) = -data(index, 12);

  policy_joints(0) = data(index, 8) - 0.4;
  policy_joints(1) = data(index, 9);
  policy_joints(2) = data(index, 10);
  policy_joints(3) = data(index, 11);
  policy_joints(4) = data(index, 12);
  policy_joints = low_pass_filter_alpha * pre_policy_joints + (1 - low_pass_filter_alpha) * policy_joints;
  pre_policy_joints = policy_joints;
//  policy_command_.joints = smuro_msgs::jointPosFromEigen(smuro_robotics_->LimitJoint(policy_joints));
  policy_command_.joints = smuro_msgs::jointPosFromEigen(policy_joints);
// publish  commands
  demo_pose_pub_.publish(demo_pose_);
  demo_command_pub_.publish(demo_command_);
  policy_pose_pub_.publish(policy_pose_);
  policy_command_pub_.publish(policy_command_);
  // show index if index%100=0
  if (index % 100 == 0) {
    ROS_INFO("index: %d", index);
  }
  index += 1;
  if (index == 2400-1) {
    index = 0;
  }
}
