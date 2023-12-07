// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

using namespace std;

namespace kdl_control_tools
{

// Template trajectory contains common interface of different trajectory types.
template<class PointType, class VelType, class AccType>
class Trajectory
{

  private:

    vector<double> times_;
    vector<PointType> points_;
    vector<VelType> velocities_;
    vector<AccType> accelerations_;

  public:
    inline Trajectory(){};
  	inline Trajectory(const Trajectory<PointType,VelType,AccType>& traj){
      // copy each field
      setTrajectory(
        traj.getTimeArray(),
        traj.getPointArray(),
        traj.getVelocityArray(),
        traj.getAccelerationArray()
      );
    }
  	inline ~Trajectory(){};

    // getters and setters
    inline PointType getPoint(int index) const{
      return points_[index];
    }
    inline VelType getVelocity(int index) const{
      return velocities_[index];
    }
    inline AccType getAcceleration(int index) const{
      return accelerations_[index];
    }
    inline double getTime(int index) const{
      return times_[index];
    }
    inline int getLength() const{
      return times_.size();
    }
    // t_traj: time since the beginning of the execution
    inline int getIndex(double t_traj) const{
      // the first step equals to the step size
      double step_size = this->getTime(1);
      // The addition of 0.5 causes the truncation to produce a rounding effect
      return int((t_traj / step_size)+0.5);
    }

    inline vector<double> getTimeArray() const{
      return times_;
    }
    inline vector<PointType> getPointArray() const{
      return points_;
    }
    inline vector<VelType> getVelocityArray() const{
      return velocities_;
    }
    inline vector<AccType> getAccelerationArray() const{
      return accelerations_;
    }

    // setters
    inline void setTrajectory(const vector<double>& ts, const vector<PointType>& xs, const vector<VelType>& xds, const vector<AccType>& xdds){
      times_ = ts;
      points_ = xs;
      velocities_ = xds;
      accelerations_ = xdds;
    }
    inline void setPoint(int index, const PointType new_val ){
      points_[index] = new_val;
    }
    inline void setVelocity(int index, const VelType new_val ){
      velocities_[index] = new_val;
    }
    inline void setAcceleration(int index, const AccType new_val ){
      accelerations_[index] = new_val;
    }
    inline void setTime(int index, double new_val){
      times_[index] = new_val;
    }

    inline void setTimeArray(const vector<double> new_arr ){
      times_ = new_arr;
    }
    inline void setPointArray(const vector<PointType> new_arr ){
      points_ = new_arr;
    }
    inline void setVelocityArray(const vector<VelType> new_arr ){
      velocities_ = new_arr;
    }
    inline void setAccelerationArray(const vector<AccType> new_arr ){
      accelerations_ = new_arr;
    }

    inline void setLength(int size){
      times_.resize(size);
      points_.resize(size);
      velocities_.resize(size);
      accelerations_.resize(size);
    }
};

// Specific trajectory types
using JointTrajectory = Trajectory<KDL::JntArray, KDL::JntArray, KDL::JntArray>;
using CartesianTrajectory = Trajectory<KDL::Frame, KDL::Twist, KDL::Wrench>;

} // end namespace kdl_control_tools

#endif // TRAJECTORY_H
