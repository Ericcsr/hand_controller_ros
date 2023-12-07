// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "topic_connector/joint_state_connector.h"

using namespace std;
using namespace topic_connector;

/*********************************************************************
* Get two JointState streams and produce a new JointState stream
*********************************************************************/
JointStateConnector::JointStateConnector(
      const int joint_count,const double period, const string name_prefix){

  setJointCount(joint_count);
  setPeriod(period);
  setNamePrefix(name_prefix);
}
JointStateConnector::~JointStateConnector(){

}
/*********************************************************************
* initiate communication, start timer loop
*********************************************************************/
void JointStateConnector::start(ros::NodeHandle &nh){
  // create joint state listeners
  js_sub1_ = nh.subscribe<sensor_msgs::JointState>( "/in1", 3, &JointStateConnector::jointStateCallback1_, this, ros::TransportHints().tcpNoDelay());
  js_sub2_ = nh.subscribe<sensor_msgs::JointState>( "/in2", 3, &JointStateConnector::jointStateCallback2_, this, ros::TransportHints().tcpNoDelay());

  // create torque publisher
  js_publisher_ = nh.advertise<sensor_msgs::JointState>("/out", 3);

  // start timer loop
  update_timer_ = nh.createTimer(ros::Duration(period_),
                    &JointStateConnector::timerCallback_, this);

}
/*********************************************************************
* calls update periodically
*********************************************************************/
void JointStateConnector::timerCallback_(const ros::TimerEvent&)  {
  // update as specified in child class
  update();
}
/*********************************************************************
* do cleaning
*********************************************************************/
void JointStateConnector::finish(){
  // send an empty JS message
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(joint_count_);
  msg.velocity.resize(joint_count_);
  msg.effort.resize(joint_count_);

  for (int j=0; j < joint_count_; j++){
    msg.name.push_back(name_prefix_+to_string(j+1));
  }

  js_publisher_.publish(msg);
}
/*********************************************************************
* simply update the joint state 1
*********************************************************************/
void JointStateConnector::jointStateCallback1_(const sensor_msgs::JointState::ConstPtr &msg){
  js_msg1_ = msg;
}
/*********************************************************************
* simply update the joint state 2
*********************************************************************/
void JointStateConnector::jointStateCallback2_(const sensor_msgs::JointState::ConstPtr &msg){
  js_msg2_ = msg;
}
/*********************************************************************
* Getters and setters
*********************************************************************/
// *********** getters
double JointStateConnector::getPeriod() const {
  return period_; }
int JointStateConnector::getJointCount() const {
  return joint_count_; }
string JointStateConnector::getNamePrefix() const {
  return name_prefix_; }
// setters
void JointStateConnector::setPeriod(const double period){
  if(period < 0) ROS_WARN("Period shouldn't be negative!");
  period_ = period; }
void JointStateConnector::setJointCount(const int count){
  if(count < 0) ROS_WARN("Joint count shouldn't be negative!");
  joint_count_ = count; }
void JointStateConnector::setNamePrefix(const string prefix){
  name_prefix_ = prefix; }
