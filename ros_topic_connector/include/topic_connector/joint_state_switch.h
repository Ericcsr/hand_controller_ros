// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef JOINT_STATE_SWITCH_H_
#define JOINT_STATE_SWITCH_H_

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <topic_connector/joint_state_connector.h>

using namespace std;

namespace topic_connector{

/**
* Get two JointState streams and select the prior one if exists.
**/
class JointStateSwitch : public JointStateConnector{

protected:

  bool is_source1_;

  bool isActiveMsg_(sensor_msgs::JointState::ConstPtr &msg);

public:

  /**
  *   This node subscribes to two joint state topics ("in1" and "in2")
  *   that contain torque info. It publishes the torque from "in1"
  *   unless "in2" publishes non-zero torques.
  *   So "in2" has priority over "in1".
  **/
	JointStateSwitch(const int joint_count=0, const double period=0.005, const string name_prefix="joint_");
  ~JointStateSwitch();

  virtual void update();

};
} // end namespace topic_connector

#endif /* JOINT_STATE_SWITCH_H_*/
