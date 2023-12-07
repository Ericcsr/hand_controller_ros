// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef MULTI_ROBOT_TRAJECTORY_H
#define MULTI_ROBOT_TRAJECTORY_H

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_control_tools/trajectory.h>

using namespace std;

namespace kdl_control_tools
{

// Template trajectory contains common interface of different trajectory types.
template<class PointType, class VelType, class AccType>
class MultiRobotTrajectory
{

  protected:

    vector<Trajectory<PointType, VelType, AccType> > trajectories_;
  public:

    inline MultiRobotTrajectory(int robot_count=1){
      trajectories_.resize(robot_count);
    };
  	inline MultiRobotTrajectory(const MultiRobotTrajectory<PointType,VelType,AccType>& mrt){
      // copy each field
      this->trajectories_ = mrt.trajectories_;
    }
  	inline ~MultiRobotTrajectory(){};

    inline Trajectory<PointType,VelType,AccType>& operator[] (int index){
      return trajectories_[index];
    }

    // size methods
    inline int resize(int robot_count){
      trajectories_.resize(robot_count);
    }
    inline int size() const{
      return trajectories_.size();
    }

    // getters and setters
    inline vector<PointType> getPoint(int index) const{
      vector<PointType> points;
      // put points of each trajectory into a vector
      for(int i=0; i < this->size(); i++)
        points.push_back(trajectories_[i].getPoint(index));
      return points;
    }
    inline vector<VelType> getVelocity(int index) const{
      vector<VelType> velocities;
      // put velocities of each trajectory into a vector
      for(int i=0; i < this->size(); i++)
        velocities.push_back(trajectories_[i].getVelocity(index));
      return velocities;
    }
    inline vector<AccType> getAcceleration(int index) const{
      vector<AccType> accelerations;
      // put accelerations of each trajectory into a vector
      for(int i=0; i < this->size(); i++)
        accelerations.push_back(trajectories_[i].getAcceleration(index));
      return accelerations;
    }
    inline double getTime(int index) const{
      return trajectories_[0].getTime(index);
    }
    inline int getLength() const{
      return trajectories_[0].getLength();
    }
    // t_traj: time since the beginning of the execution
    inline int getIndex(double t_traj) const{
      return trajectories_[0].getIndex(t_traj);
    }

    inline vector<double> getTimeArray() const{
      return trajectories_[0].getTimeArray();
    }

    inline void setTime(int index, double new_val){
      for(int i=0; i < this->size(); i++)
        trajectories_[i].setTime(index, new_val);
    }

    inline void setTimeArray(const vector<double> new_arr ){
      for(int i=0; i < this->size(); i++)
        trajectories_[i].setTimeArray(index, new_arr);
    }

    inline void setLength(int size){
      for(int i=0; i < this->size(); i++)
        trajectories_[i].setLength(size);
    }
};

// Specific trajectory types
using MultiJointTrajectory = MultiRobotTrajectory<KDL::JntArray, KDL::JntArray, KDL::JntArray>;
using MultiCartesianTrajectory = MultiRobotTrajectory<KDL::Frame, KDL::Twist, KDL::Wrench>;
using HandTrajectory = MultiCartesianTrajectory;

} // end namespace kdl_control_tools

#endif // MULTI_ROBOT_TRAJECTORY_H
