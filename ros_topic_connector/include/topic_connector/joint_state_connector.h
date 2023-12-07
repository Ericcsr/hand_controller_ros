// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef JOINT_STATE_CONNECTOR_H_
#define JOINT_STATE_CONNECTOR_H_

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

using namespace std;

namespace topic_connector{

/**
* Get two JointState streams and produce a new JointState stream.
**/
class JointStateConnector{

protected:

  // params
  double period_;
  int joint_count_;
  string name_prefix_;

  // communication objects
  ros::Publisher js_publisher_;
  ros::Subscriber js_sub1_;
  ros::Subscriber js_sub2_;

  sensor_msgs::JointState::ConstPtr js_msg1_;
  sensor_msgs::JointState::ConstPtr js_msg2_;

  ros::Timer update_timer_;

  virtual void jointStateCallback1_(const sensor_msgs::JointState::ConstPtr &msg);
  virtual void jointStateCallback2_(const sensor_msgs::JointState::ConstPtr &msg);

  // calls update periodically
  void timerCallback_(const ros::TimerEvent&);

public:

	JointStateConnector(const int joint_count=0, const double period=0.005, const string name_prefix="joint_");
  ~JointStateConnector();

  // initiate communication, start timer loop
  void start(ros::NodeHandle &nh);
  // connect two JointState messages and publish
  virtual void update() = 0;
  // do cleaning
  virtual void finish();

  // getters and setters
  double getPeriod() const;
  int getJointCount() const;
  string getNamePrefix() const;

  void setPeriod(const double period);
  void setJointCount(const int count);
  void setNamePrefix(const string prefix);

};
} // end namespace topic_connector

#endif /* JOINT_STATE_CONNECTOR_H_*/
