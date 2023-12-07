// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef JOINT_STATE_LIMITER_H_
#define JOINT_STATE_LIMITER_H_

#include <limits>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <topic_connector/joint_state_connector.h>

using namespace std;

namespace topic_connector{

/**
* Get a JointState stream and ensure that it satisfies a limit.
**/
class JointStateLimiter : public JointStateConnector{

enum LimitTarget {
  limitPosition,
  limitVelocity,
  limitTorque
};

protected:
  // TODO: specific limit per each joint

  // vectors holding the upper and lower limits
  // indexed by LimitTarget
  vector<double> lim_upper_vec_;
  vector<double> lim_low_vec_;

  bool checkLimit_(JointStateLimiter::LimitTarget target);

public:


	JointStateLimiter(const int joint_count=0, const double period=0.005, const string name_prefix="joint_");
  ~JointStateLimiter();

  void readLimitsFromParams();

  virtual void update();

  // getters and setters
  double getUpperLimit(LimitTarget target);
  double getLowerLimit(LimitTarget target);

  void setUpperLimit(LimitTarget target, double value);
  void setLowerLimit(LimitTarget target, double value);
  void setSymmetricLimit(LimitTarget target, double value);

};
} // end namespace topic_connector


#endif /* JOINT_STATE_LIMITER_H_*/
