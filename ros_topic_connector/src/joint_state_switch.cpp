// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "topic_connector/joint_state_switch.h"

using namespace std;
using namespace topic_connector;

/*********************************************************************
* Get two JointState streams and select the prior one if exists.
*********************************************************************/
JointStateSwitch::JointStateSwitch(
  const int joint_count, const double period, const string name_prefix):
JointStateConnector(joint_count,period, name_prefix)
{
    is_source1_ = true;
}
JointStateSwitch::~JointStateSwitch(){

}
/*********************************************************************
* Check if the prior channel exists and publish if so switch to it
*********************************************************************/
void JointStateSwitch::update(){
  // decide on switch
  sensor_msgs::JointState::ConstPtr msg_switch;

  if(isActiveMsg_(js_msg2_)){
    // inform change
    if(is_source1_)
      ROS_INFO("JointState Switch: publishing torque source 2");
    is_source1_ =false;

    msg_switch = js_msg2_;
  } else{
    // inform change
    if(!is_source1_)
      ROS_INFO("JointState Switch: publishing torque source 1");
    is_source1_=true;

    msg_switch = js_msg1_;
  }

  // publish the selected msg
  if(msg_switch)
    js_publisher_.publish(msg_switch);
}
/*********************************************************************
* Check if the message is existing and nonzero
*********************************************************************/
bool JointStateSwitch::isActiveMsg_(sensor_msgs::JointState::ConstPtr &msg){
  if(!msg || (int)msg->position.size() != joint_count_) return false;

  // check time out (2 x period)
  ros::Time t_now = ros::Time::now();
  ros::Time t_msg = msg->header.stamp;
  double dt = (t_now - t_msg).sec + 1e-9 * (t_now - t_msg).nsec;
  if(dt > 2 * period_)
    return false;

  // otherwise it's active
  return true;
}
