// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "topic_connector/joint_state_adder.h"

using namespace std;
using namespace topic_connector;

/*********************************************************************
* Get two JointState streams and publish their sum.
*********************************************************************/
JointStateAdder::JointStateAdder(
  const int joint_count, const double period, const string name_prefix):
JointStateConnector(joint_count,period, name_prefix)
{

}

JointStateAdder::~JointStateAdder()  {

}
/*********************************************************************
* Produce the sum of last js messages and publish
*********************************************************************/
void JointStateAdder::update()  {
  // null pointer safety
  if (!js_msg1_) createEmptyMsg_(js_msg1_);
  if (!js_msg2_) createEmptyMsg_(js_msg2_);

  // create a joint state message and publish
  sensor_msgs::JointState msg;

  msg.header.stamp = ros::Time::now();

  msg.position.resize(joint_count_);
  msg.velocity.resize(joint_count_);
  msg.effort.resize(joint_count_);

  for (int j=0; j < joint_count_; j++){

    msg.position[j] = js_msg1_->position[j] + js_msg2_->position[j];
    msg.velocity[j] = js_msg1_->velocity[j] + js_msg2_->velocity[j];
    msg.effort[j] = js_msg1_->effort[j] + js_msg2_->effort[j];

    // names of joints
    msg.name.push_back(name_prefix_+to_string(j+1));
  }

  js_publisher_.publish(msg);

}
/*********************************************************************
* Create an empty message to avoid null pointer error
*********************************************************************/
void JointStateAdder::createEmptyMsg_(sensor_msgs::JointState::ConstPtr &msg){
  // create the object
  sensor_msgs::JointState* empty_one = new sensor_msgs::JointState();

  empty_one->position.resize(joint_count_);
  empty_one->velocity.resize(joint_count_);
  empty_one->effort.resize(joint_count_);

  // assign to pointer
  msg = sensor_msgs::JointState::ConstPtr(empty_one);
}
