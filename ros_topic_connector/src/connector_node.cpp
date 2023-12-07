// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

/*********************************************************************
* A generic ros node to run a JointState connector
*********************************************************************/
#include <ros/ros.h>
#include <signal.h>

#include <sensor_msgs/JointState.h>

#include <topic_connector/joint_state_adder.h>
#include <topic_connector/joint_state_switch.h>
#include <topic_connector/joint_state_limiter.h>

using namespace std;
using namespace topic_connector;

// ros params
string connector_type_;
int joint_count_;
double period_;

JointStateConnector *connector_obj_;

bool getParams(ros::NodeHandle nh);
void sigintCallback(int sig);

int main(int argc, char** argv){

  ros::init(argc, argv, "connector_node");

  ros::NodeHandle nh("~");

  if (!getParams(nh)) return -1;

  // create the appropriate object
  if( connector_type_ == "adder"){
    connector_obj_ = new JointStateAdder(joint_count_, period_, "joint_");

  }else if( connector_type_ == "switch"){
    connector_obj_ = new JointStateSwitch(joint_count_, period_, "joint_");

  }else if( connector_type_ == "limiter"){
    connector_obj_ = new JointStateLimiter(joint_count_, period_, "joint_");
    ((JointStateLimiter*)connector_obj_)->readLimitsFromParams();
  }else{
    ROS_ERROR("Connector node: undefined type %s", connector_type_.c_str());
    return -1;
  }

  // ros shutdown handler
  signal(SIGINT, sigintCallback);

  // call start procedure
  connector_obj_->start(nh);

  ros::spin();

  delete connector_obj_;

  return 0;
}

// get external parameters if possible
bool getParams(ros::NodeHandle nh){

  if(!nh.getParam("connector_type", connector_type_) ){
    ROS_ERROR("Connector node: Can't find connector_type param.");
    return false;
  }
  ROS_DEBUG("Connector node: type is %s.", connector_type_.c_str());

  if(!nh.getParam("joint_count", joint_count_) ){
    ROS_ERROR("Connector node: Can't find joint_count param.");
    return false;
  }
  ROS_DEBUG("Connector node: %d joints.", joint_count_);

  if(!nh.getParam("period", period_) ){
    ROS_ERROR("Connector node: Can't find period param.");
    return false;
  }
  ROS_DEBUG("Connector node: publishes every %.3f seconds.", period_);

  return true;

}

void sigintCallback(int sig)
{
  // inform user
  ROS_INFO("Connector node: cleaning up");

  // call object's finish procedure
  connector_obj_->finish();

  // spin once to hand communication
  ros::spinOnce();

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}
