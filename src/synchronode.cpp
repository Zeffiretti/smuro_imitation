//
// Created by smurolap on 2022/2/10.
//

/*************************************************************
 * File Name : synchronode.cpp
 * Author    : ZEFFIRETTI, HESH
 * College   : Beijing Institute of Technology
 * E-Mail    : zeffiretti@bit.edu.cn, hiesh@mail.com
**********************************************************/

#include <ros/ros.h>
#include <smuro_imitation/synchrocontroller/synchrocontroller.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Smuro_Synchro_Control_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  SynchroController controller(&nh, &nh_private);

  ros::spin();
  return 0;
}
