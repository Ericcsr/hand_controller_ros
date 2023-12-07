// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef JOINT_STATE_ADDER_H_
#define JOINT_STATE_ADDER_H_

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <topic_connector/joint_state_connector.h>

using namespace std;

namespace topic_connector{

/**
* Get two JointState streams and publish their sum.
**/
class JointStateAdder : public JointStateConnector{

protected:

	void createEmptyMsg_(sensor_msgs::JointState::ConstPtr &msg);

public:

	JointStateAdder(const int joint_count=0, const double period=0.005, const string name_prefix="joint_");
  ~JointStateAdder();

  virtual void update();

};
} // end namespace topic_connector

#endif /* JOINT_STATE_ADDER_H_*/
