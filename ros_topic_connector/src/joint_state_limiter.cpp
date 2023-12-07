// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "topic_connector/joint_state_limiter.h"

using namespace std;
using namespace topic_connector;

/*********************************************************************
* Get a JointState stream and ensure that it satisfies a limit.
*********************************************************************/
JointStateLimiter::JointStateLimiter(
  const int joint_count, const double period, const string name_prefix):
JointStateConnector(joint_count,period, name_prefix)
{
  // Terminate if
  static_assert(std::numeric_limits<float>::is_iec559, "JointStateLimiter: IEEE 754 required");

  // unlimited
  lim_upper_vec_.resize(3, INFINITY);
  lim_low_vec_.resize(3, -INFINITY);
}
JointStateLimiter::~JointStateLimiter()
{

}
/*********************************************************************
* Get a JointState stream and ensure that it satisfies a limit.
*********************************************************************/
void JointStateLimiter::update()
{
  if(!js_msg1_) return;

  for(int t=0; t <= limitTorque; t++){
    if(!checkLimit_((LimitTarget)t)){
      // shutdown procedure
      finish();
      ros::spinOnce();
      ros::shutdown();
      return;
    }
  }

  // if limits are OK, publish as it is
  js_publisher_.publish(js_msg1_);
}
/*********************************************************************
* Return false if limit is not satisfied
*********************************************************************/
bool JointStateLimiter::checkLimit_(LimitTarget target) {
  // get proper state vector
  const vector<double> * state;
  switch (target) {
    case limitPosition:
      state = &(js_msg1_->position);
    case limitVelocity:
      state = &(js_msg1_->velocity);
    case limitTorque:
      state = &(js_msg1_->effort);
  }

  // upper limit
  for(int j=0; j<state->size(); j++)
    if (state->at(j) >= lim_upper_vec_[target])
      return false;

  // lower limit
  for(int j=0; j<state->size(); j++)
    if (state->at(j) <= lim_low_vec_[target])
      return false;

  return true;
}
/*********************************************************************
* Read limits from ros params
*********************************************************************/
void JointStateLimiter::readLimitsFromParams(){

  double lim_val;
  // position
  if(ros::param::get("~limits/position/upper", lim_val) ){
    ROS_DEBUG("Connector node: limits/position/upper=%.3f.", lim_val);
    setUpperLimit(limitPosition, lim_val);
  }
  if(ros::param::get("~limits/position/lower", lim_val) ){
    ROS_DEBUG("Connector node: limits/position/lower=%.3f.", lim_val);
    setLowerLimit(limitPosition, lim_val);
  }
  // velocity
  if(ros::param::get("~limits/velocity/upper", lim_val) ){
    ROS_DEBUG("Connector node: limits/velocity/upper=%.3f.", lim_val);
    setUpperLimit(limitVelocity, lim_val);
  }
  if(ros::param::get("~limits/velocity/lower", lim_val) ){
    ROS_DEBUG("Connector node: limits/velocity/lower=%.3f.", lim_val);
    setLowerLimit(limitVelocity, lim_val);
  }
  // torque
  if(ros::param::get("~limits/torque/upper", lim_val) ){
    ROS_DEBUG("Connector node: limits/torque/upper=%.3f.", lim_val);
    setUpperLimit(limitTorque, lim_val);
  }
  if(ros::param::get("~limits/torque/lower", lim_val) ){
    ROS_DEBUG("Connector node: limits/torque/lower=%.3f.", lim_val);
    setLowerLimit(limitTorque, lim_val);
  }
}
/*********************************************************************
* Getters and setters
*********************************************************************/
double JointStateLimiter::getUpperLimit(LimitTarget target) {
  return lim_upper_vec_[target]; }
double JointStateLimiter::getLowerLimit(LimitTarget target){
  return lim_low_vec_[target]; }
// setters
void JointStateLimiter::setUpperLimit(LimitTarget target, double value) {
  lim_upper_vec_[target] = value;
}
void JointStateLimiter::setLowerLimit(LimitTarget target, double value) {
  lim_low_vec_[target] = value;
}
void JointStateLimiter::setSymmetricLimit(LimitTarget target, double value) {
  setUpperLimit(target, value);
  setLowerLimit(target, -value);
}
